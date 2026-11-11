"""Tests for the gr-roboracer-docs skill's docs_check.py (CHECK-2460..2462)."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DOCS_CHECK = (
    REPO_ROOT / ".claude/skills/gr-roboracer-docs/scripts/docs_check.py"
)


def _run(tmp_path: Path, readme: str, page: str) -> tuple[int, str]:
    (tmp_path / "docs").mkdir()
    (tmp_path / "docs/agents").mkdir()
    (tmp_path / "docs/agents/x.md").write_text("# agent\n")
    (tmp_path / "README.md").write_text(readme)
    (tmp_path / "docs/x.md").write_text(page)
    done = subprocess.run(
        [sys.executable, "-I", str(DOCS_CHECK), str(tmp_path)],
        capture_output=True,
        text=True,
        check=False,
    )
    return done.returncode, done.stdout


def test_check_2460_user_page_may_name_wsl_and_windows(tmp_path: Path) -> None:
    """CHECK-2460: a user page that says WSL or Windows passes (R-b: user
    docs may document the Windows GUI sim and WSL steps)."""
    code, out = _run(
        tmp_path,
        "# R\n",
        "# X\n\nOn Windows, open WSL2 and start Docker Desktop.\n",
    )
    assert code == 0, out


def test_check_2461_user_page_linking_agent_doc_fails(tmp_path: Path) -> None:
    """CHECK-2461: a user page linking docs/agents/ still fails."""
    code, out = _run(tmp_path, "# R\n", "# X\n\nSee [a](agents/x.md).\n")
    assert code == 1
    assert "links an agent doc" in out


def test_check_2462_broken_link_or_anchor_fails(tmp_path: Path) -> None:
    """CHECK-2462: a missing link target and a missing anchor both fail."""
    code, out = _run(
        tmp_path, "# R\n", "# X\n\n[a](nope.md) and [b](#no-such)\n"
    )
    assert code == 1
    assert "missing target: nope.md" in out
    assert "missing anchor: #no-such" in out
