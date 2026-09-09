"""Make the racing_test_keywords library importable without installing it.

It is a plain PYTHONPATH package, not an ament package — it needs no ROS
build step of its own, only the ROS overlay `scripts/check.sh` already
sources before running these tests.
"""

import sys
from pathlib import Path

LIB_ROOT = Path(__file__).resolve().parents[1] / "lib"
if str(LIB_ROOT) not in sys.path:
    sys.path.insert(0, str(LIB_ROOT))
