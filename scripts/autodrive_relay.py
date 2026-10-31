"""TCP relay for the AutoDRIVE route (scripts/autodrive_route.sh).

Runs in a throwaway container on the compose network, publishing
127.0.0.1:4567 through Docker Desktop, and pipes each connection to the
server in `dev` (which publishes nothing: docker/ is not ours to change).
TCP_NODELAY is set on both legs. Environment: UP_HOST, UP_PORT, optional
NODELAY (default 1).
"""

import asyncio
import os
import socket

UPSTREAM = (os.environ["UP_HOST"], int(os.environ["UP_PORT"]))
NODELAY = os.environ.get("NODELAY", "1") == "1"


def _nodelay(writer) -> None:
    if NODELAY:
        sock = writer.get_extra_info("socket")
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)


async def _pipe(reader, writer) -> None:
    try:
        while data := await reader.read(65536):
            writer.write(data)
            await writer.drain()
    except OSError:
        pass
    finally:
        writer.close()


async def _handle(reader, writer) -> None:
    try:
        up_reader, up_writer = await asyncio.open_connection(*UPSTREAM)
    except OSError:
        writer.close()
        return
    _nodelay(writer)
    _nodelay(up_writer)
    await asyncio.gather(_pipe(reader, up_writer), _pipe(up_reader, writer))


async def _main() -> None:
    server = await asyncio.start_server(_handle, "0.0.0.0", 4567)
    await server.serve_forever()


if __name__ == "__main__":
    asyncio.run(_main())
