"""ROS shell of front end A: our Socket.IO server for the AutoDRIVE sim.

The Unity sim connects here as a client, once, and sends `Bridge` frames; this
node answers each with throttle, steering and reset, and publishes the
simulator contract's topics (racing_sim_adapter/contract.hpp) under the node
name `racing_sim`. Every conversion is `bridge.BridgeSession` (ROS-free).

Threading (validated in plan step 1, no monkey-patching): the gevent
WSGIServer runs on the main thread and calls the Bridge handler; the rclpy
executor spins in a daemon thread for `/drive` and the services. Publishing
from the handler is safe; the only shared state is the latest `/drive`
(under a lock) and the session's reset flag (one bool).

Stamps are the node clock at frame receipt (ADR 0006, `free_running`),
exactly as the official devkit stamps; the sim sends none.
"""

import faulthandler
import signal
import sys
import threading
import time

import gevent
import rclpy
import socketio
from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler
from rclpy.executors import SingleThreadedExecutor

from racing_sim_autodrive import bridge, core
from racing_sim_autodrive.contract_node import (  # noqa: F401
    SCAN_FRAME_ID,
    SENSOR_QOS,
    STATE_QOS,
    VEHICLE_PARAMETERS,
    ContractNode,
    _ignore_signals,
)

# A connected sim silent for this long is logged once (gotcha 29, 30).
STALL_TIMEOUT_S = 0.5
# The executor thread's loop: wait this long for work, then yield the GIL.
SPIN_TIMEOUT_S = 0.05
SPIN_YIELD_S = 0.001


class AutoDriveServer(ContractNode):
    def __init__(self) -> None:
        # `_build_session` (called by the base constructor) declares host,
        # port and dt_source, since the Node exists by then.
        super().__init__()
        self._host = self._text("host")
        self._port = (
            self.get_parameter("port").get_parameter_value().integer_value
        )
        self._sid: str | None = None
        self._stall = bridge.StallWatch(STALL_TIMEOUT_S)
        self.sio = socketio.Server(async_mode="gevent")
        self.sio.on("connect", self._on_connect)
        self.sio.on("disconnect", self._on_disconnect)
        self.sio.on("Bridge", self._on_bridge)

    def _build_session(self, vehicle, world_to_map, track):
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 4567)  # config/autodrive/simulator.yaml
        self.declare_parameter("dt_source", "lap_time")
        return bridge.BridgeSession(
            vehicle,
            world_to_map,
            track,
            dt_source=self._text("dt_source"),
        )

    # ---- Socket.IO side (gevent greenlet, main thread) ----

    def _on_connect(self, sid, environ) -> None:
        self._sid = sid
        self._stall.connected(time.monotonic())
        self.get_logger().info(f"sim connected: {sid}")

    def _on_disconnect(self, sid) -> None:
        self._stall.disconnected()
        self.get_logger().error(
            f"sim disconnected: {sid}. It never reconnects (gotcha 30): "
            "restart the sim."
        )

    def _on_bridge(self, sid, data) -> None:
        self._stall.entered()
        try:
            self._handle_bridge(sid, data)
        finally:
            self._stall.exited(time.monotonic())

    def _handle_bridge(self, sid, data) -> None:
        now = self.get_clock().now()
        stamp = now.to_msg()
        with self._drive_lock:
            drive = self._drive
        try:
            step = self._session.on_frame(
                data, drive, received_at=now.nanoseconds * 1e-9
            )
        except core.FrameError as error:
            # the sim is never left waiting (gotcha 29)
            self.get_logger().error(f"bad Bridge frame: {error}")
            self.sio.emit("Bridge", bridge.fallback_reply(), to=sid)
            self._stall.emitted()
            return
        self._publish(stamp, step)
        self.sio.emit("Bridge", step.reply, to=sid)
        self._stall.emitted()

    def _watch_stalls(self) -> None:
        """Log one WARNING and one all-thread stack dump per stall."""
        while True:
            gevent.sleep(STALL_TIMEOUT_S / 5)
            report = self._stall.check(time.monotonic())
            if report is not None:
                self.get_logger().warning(f"stall: {report}")
                faulthandler.dump_traceback(file=sys.stderr, all_threads=True)

    def serve(self) -> None:
        """Run the Socket.IO server until SIGINT/SIGTERM (main thread)."""
        server = pywsgi.WSGIServer(
            (self._host, self._port),
            socketio.WSGIApp(self.sio),
            handler_class=WebSocketHandler,
        )
        # Bound before the handlers and the log line: a signal arriving ahead
        # of start() would stop an unstarted server and crash serve_forever.
        server.start()
        gevent.spawn(self._watch_stalls)
        gevent.signal_handler(signal.SIGINT, server.stop)
        gevent.signal_handler(signal.SIGTERM, server.stop)
        self.get_logger().info(f"listening on {self._host}:{self._port}")
        if not rclpy.ok():
            # rclpy's own SIGINT handler (installed by rclpy.init) fired
            # before ours existed: nothing would stop serve_forever.
            server.stop()
            return
        server.serve_forever()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoDriveServer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stopping = threading.Event()

    def spin() -> None:
        # Not executor.spin(): under a /drive flood that loop never blocks,
        # holds the GIL, and the gevent hub on the main thread gets it once
        # per switch interval, so replies drain at ~20/s (SIMAD-2080). One
        # callback, then sleep: the sleep releases the GIL for the hub.
        try:
            while rclpy.ok() and not stopping.is_set():
                executor.spin_once(timeout_sec=SPIN_TIMEOUT_S)
                time.sleep(SPIN_YIELD_S)
        except Exception:
            if rclpy.ok():
                raise

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        node.serve()
    finally:
        # The launch forwards SIGINT and the caller's group signal is another,
        # so a second one arrives while this runs. A KeyboardInterrupt here
        # kills the process with -2 ("process has died"), and rclpy.shutdown
        # restores the default handler, so ignore it again after.
        _ignore_signals()
        stopping.set()
        thread.join(timeout=5.0)  # a spin left running aborts at exit
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        _ignore_signals()


if __name__ == "__main__":
    main()
