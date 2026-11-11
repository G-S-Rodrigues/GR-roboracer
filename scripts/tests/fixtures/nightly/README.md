# Nightly evidence (CHECK-2378)

Trimmed, gzipped excerpts of tier-4 localization recordings from red nightlies. Each keeps the manifest
line and only the listed topics, so `test_nightly_evidence.py` replays the real numbers.

| File | Run / job | Source recording | Window (sim s) | Topics |
|---|---|---|---|---|
| `acc4040-raceline-20261104-slam2026.jsonl.gz` | nightly 2026-11-04, slam seed 2026 | `robot/localization/amcl-closed-loop-optimized_raceline.jsonl` | 61.69-67.19 (freeze 64.69) | truth state, error |
| `acc4040-raceline-20261110-slam7.jsonl.gz` | run 42802073362, job 129781861078 (slam seed 7) | same file name | 107.51-113.01 (freeze 110.51) | truth state, error |
| `acc4510-mola-20261110-slam7.jsonl.gz` | run 42802073362, job 129781861078 | `mola-open-loop.jsonl` | first 5 s | truth odom, estimate, error |

Trim: `python3 trim.py SOURCE.jsonl T0 T1 OUT.jsonl.gz TOPIC...` keeps line 0 (manifest) and each
listed-topic event stamped T0 <= t <= T1, gzipped. Re-download sources with
`gh run download 42802073362 -n logs-slam-seed-7`.
