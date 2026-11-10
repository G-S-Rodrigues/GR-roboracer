"""usage: n1trim.py SOURCE.jsonl T0 T1 OUT.jsonl.gz TOPIC [TOPIC ...]
Keeps the manifest line and the listed topics' events stamped T0 <= t <= T1."""
import gzip, io, json, sys
src, t0, t1, out, *topics = sys.argv[1:]
t0, t1 = float(t0), float(t1)
keep = [f'"topic":"{t}"' for t in topics]
n = 0
with open(src) as fh, open(out, "wb") as raw, gzip.GzipFile(fileobj=raw, mode="wb", compresslevel=9) as gz, io.TextIOWrapper(gz, encoding="utf-8") as dst:
    for i, line in enumerate(fh):
        if i == 0:
            dst.write(line); continue
        if not any(k in line for k in keep): continue
        try: r = json.loads(line)
        except ValueError: break
        t = r["stamp"]["sec"] + r["stamp"]["nanosec"] * 1e-9
        if t0 <= t <= t1:
            dst.write(line); n += 1
print(out, n)
