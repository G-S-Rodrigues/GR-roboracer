"""Rebuild H.264 from an MP4 whose moov was never written.
usage: salvage_mp4.py broken.mp4 reference.mp4 out.h264
Reference = short clip encoded with identical settings (supplies SPS/PPS)."""

import pathlib
import struct
import sys

broken, ref, out = sys.argv[1:4]
r = pathlib.Path(ref).read_bytes()
i = r.index(b"avcC") + 4
nal_len_size = (r[i + 4] & 3) + 1
nsps = r[i + 5] & 0x1F
p = i + 6
hdr = b""
for _ in range(nsps):
    n = struct.unpack(">H", r[p : p + 2])[0]
    hdr += b"\x00\x00\x00\x01" + r[p + 2 : p + 2 + n]
    p += 2 + n
npps = r[p]
p += 1
for _ in range(npps):
    n = struct.unpack(">H", r[p : p + 2])[0]
    hdr += b"\x00\x00\x00\x01" + r[p + 2 : p + 2 + n]
    p += 2 + n
b = pathlib.Path(broken).read_bytes()
j = b.index(b"mdat")
# data starts after 'mdat' (size field of 8 bytes is before it)
pos = j + 4
o = pathlib.Path(out).open("wb")  # noqa: SIM115 (streamed)
o.write(hdr)
frames = 0
while pos + 4 <= len(b):
    n = struct.unpack(">I", b[pos : pos + 4])[0]
    if n == 0 or pos + 4 + n > len(b):
        break
    nal = b[pos + 4 : pos + 4 + n]
    o.write(b"\x00\x00\x00\x01" + nal)
    if (nal[0] & 0x1F) in (1, 5):
        frames += 1
    pos += 4 + n
o.close()
print(
    "nal_len_size",
    nal_len_size,
    "frames",
    frames,
    "stopped at",
    pos,
    "of",
    len(b),
)
