"""usage: trim.py SOURCE.jsonl T0 T1 OUT.jsonl.gz TOPIC [TOPIC ...]

Keeps the manifest line and the listed topics' events stamped T0 <= t <= T1.
"""

import gzip
import json
import sys


def main() -> None:
    src, t0, t1, out, *topics = sys.argv[1:]
    low, high = float(t0), float(t1)
    keys = [f'"topic":"{topic}"' for topic in topics]
    with open(src) as fh, gzip.open(out, "wt", compresslevel=9) as dst:
        for i, line in enumerate(fh):
            if i == 0:
                dst.write(line)
                continue
            if not any(key in line for key in keys):
                continue
            try:
                record = json.loads(line)
            except ValueError:
                break
            stamp = record["stamp"]
            t = stamp["sec"] + stamp["nanosec"] * 1e-9
            if low <= t <= high:
                dst.write(line)


if __name__ == "__main__":
    main()
