"""Fixed-cadence RViz grabber: raw bgr0 frames to stdout."""

import argparse
import ctypes as c
import ctypes.util
import pathlib
import sys
import time

x = c.CDLL(ctypes.util.find_library("X11"))
ptr, ulong, uint, integer = c.c_void_p, c.c_ulong, c.c_uint, c.c_int
x.XOpenDisplay.argtypes = [c.c_char_p]
x.XOpenDisplay.restype = ptr
x.XDefaultRootWindow.argtypes = [ptr]
x.XDefaultRootWindow.restype = ulong
x.XQueryTree.argtypes = [
    ptr,
    ulong,
    c.POINTER(ulong),
    c.POINTER(ulong),
    c.POINTER(c.POINTER(ulong)),
    c.POINTER(uint),
]
x.XFetchName.argtypes = [ptr, ulong, c.POINTER(c.c_char_p)]
x.XGetGeometry.argtypes = [
    ptr,
    ulong,
    c.POINTER(ulong),
    c.POINTER(integer),
    c.POINTER(integer),
    c.POINTER(uint),
    c.POINTER(uint),
    c.POINTER(uint),
    c.POINTER(uint),
]
x.XGetImage.argtypes = [
    ptr,
    ulong,
    integer,
    integer,
    uint,
    uint,
    ulong,
    integer,
]
x.XGetImage.restype = ptr
x.XFree.argtypes = [ptr]
x.XDestroyImage.argtypes = [ptr]
x.XCloseDisplay.argtypes = [ptr]


class XImage(c.Structure):
    _fields_ = [
        ("width", integer),
        ("height", integer),
        ("xoffset", integer),
        ("format", integer),
        ("data", ptr),
        ("byte_order", integer),
        ("bitmap_unit", integer),
        ("bitmap_bit_order", integer),
        ("bitmap_pad", integer),
        ("depth", integer),
        ("bytes_per_line", integer),
        ("bits_per_pixel", integer),
        ("red_mask", ulong),
        ("green_mask", ulong),
        ("blue_mask", ulong),
    ]


ap = argparse.ArgumentParser()
ap.add_argument("--fps", type=float, default=15)
ap.add_argument("--seconds", type=float, default=5)
ap.add_argument("--frames-csv", default="frames.csv")
ap.add_argument("--geom-out", default="geom.txt")
ap.add_argument("--crop-out", default="crop.txt")
a = ap.parse_args()
d = x.XOpenDisplay(None)
if not d:
    raise SystemExit("Cannot open DISPLAY")
root = x.XDefaultRootWindow(d)
seen = set()
matches = []


def walk(window):
    if window in seen:
        return
    seen.add(window)
    name = c.c_char_p()
    if x.XFetchName(d, window, c.byref(name)) and name.value:
        if "rviz" in name.value.decode(errors="replace").lower():
            matches.append(window)
        x.XFree(name)
    children = c.POINTER(ulong)()
    count = uint()
    parent = ulong()
    rr = ulong()
    if x.XQueryTree(
        d,
        window,
        c.byref(rr),
        c.byref(parent),
        c.byref(children),
        c.byref(count),
    ):
        for i in range(count.value):
            walk(children[i])
        if children:
            x.XFree(children)


def geom(w):
    a_, b_ = integer(), integer()
    ww, hh, bb, dd = uint(), uint(), uint(), uint()
    r = ulong()
    x.XGetGeometry(
        d,
        w,
        c.byref(r),
        c.byref(a_),
        c.byref(b_),
        c.byref(ww),
        c.byref(hh),
        c.byref(bb),
        c.byref(dd),
    )
    return ww.value, hh.value


t_wait = time.time()
window = None
while window is None and time.time() - t_wait < 90:
    seen.clear()
    matches.clear()
    walk(root)
    big = [w for w in matches if geom(w)[0] >= 800 and geom(w)[1] >= 500]
    if big:
        window = max(big, key=lambda w: geom(w)[0] * geom(w)[1])
    else:
        time.sleep(0.5)
if window is None:
    raise SystemExit("No RViz X11 window of usable size")
W, H = geom(window)
pathlib.Path(a.geom_out).write_text(f"{W}x{H}\n")
cw, ch = int(W * 0.5) // 2 * 2, int(H * 0.6) // 2 * 2
pathlib.Path(a.crop_out).write_text(
    f"crop={cw}:{ch}:{int(W * 0.35)}:{int(H * 0.2)}\n"
)
out = sys.stdout.buffer
csv = pathlib.Path(a.frames_csv).open("w")  # noqa: SIM115 (streamed)
csv.write("n,wall,dup\n")
period = 1.0 / a.fps
t0 = time.time()
n = 0
last = None
while True:
    target = t0 + n * period
    now = time.time()
    if now - t0 >= a.seconds:
        break
    if now < target:
        time.sleep(target - now)
    dup = 0
    im = x.XGetImage(d, window, 0, 0, W, H, ulong(-1), 2)
    if im:
        s = c.cast(im, c.POINTER(XImage)).contents
        last = c.string_at(s.data, s.bytes_per_line * s.height)
        x.XDestroyImage(im)
    else:
        dup = 1
    if last is None:
        raise SystemExit("XGetImage failed")
    out.write(last)
    csv.write(f"{n},{time.time():.4f},{dup}\n")
    csv.flush()
    n += 1
out.flush()
csv.close()
x.XCloseDisplay(d)
