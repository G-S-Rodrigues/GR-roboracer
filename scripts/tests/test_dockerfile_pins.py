"""Guards on how docker/Dockerfile installs pip tools (the dev-stage drift).

Scope: `pip3 install` / `pip install` RUNs only. `uv pip install` lines and
the uv installer are deliberately not covered (open topic: image
reproducibility, E2).
"""

from __future__ import annotations

import re
from pathlib import Path

from scripts.dockerfile_parse import instructions as _instructions
from scripts.dockerfile_parse import segments as _segments

REPO_ROOT = Path(__file__).resolve().parents[2]
PIN_LINE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*==[0-9][0-9A-Za-z.+!-]*$")
TOOLS = {"robotframework", "robotframework-tidy", "ruff", "pre-commit"}


def _is_pip_install(segment: list[str]) -> bool:
    return (
        len(segment) >= 2
        and segment[0] in ("pip", "pip3")
        and segment[1] == "install"
    )


def _pip_runs() -> list[tuple[int, list[list[str]]]]:
    runs = []
    for number, text in _instructions():
        if not text.startswith("RUN "):
            continue
        segments = _segments(text)
        if any(_is_pip_install(s) for s in segments):
            runs.append((number, segments))
    return runs


def _requirement_files(segment: list[str]) -> list[str]:
    return [segment[i + 1] for i, t in enumerate(segment[:-1]) if t == "-r"]


def _positionals(segment: list[str]) -> list[str]:
    """Tokens after `install` that are neither options nor their values."""
    value_options = {"-r", "-c", "--requirement", "--constraint"}
    out: list[str] = []
    skip = False
    for token in segment[2:]:
        if skip:
            skip = False
        elif token in value_options:
            skip = True
        elif not token.startswith("-"):
            out.append(token)
    return out


def test_check_1330_pip_installs_are_closed_requirement_files() -> None:
    """CHECK-1330: every pip install in the Dockerfile reads one pinned
    requirements file with --no-deps and no positional requirement, and the
    same RUN runs `pip3 check`, so nothing resolves at build time and a gap
    in the pinned closure fails the build."""
    runs = _pip_runs()
    assert runs, "no pip install RUN found in docker/Dockerfile"
    problems: list[str] = []
    for number, segments in runs:
        where = f"docker/Dockerfile:{number}"
        installs = [i for i, s in enumerate(segments) if _is_pip_install(s)]
        for i in installs:
            segment = segments[i]
            if "--no-deps" not in segment:
                problems.append(f"{where}: no --no-deps")
            if len(_requirement_files(segment)) != 1:
                problems.append(f"{where}: expected exactly one -r file")
            if _positionals(segment):
                problems.append(
                    f"{where}: positional requirement(s) "
                    f"{' '.join(_positionals(segment))}"
                )
        checks = [
            i
            for i, s in enumerate(segments)
            if s[:2] in (["pip3", "check"], ["pip", "check"])
        ]
        if not checks or max(checks) < max(installs):
            problems.append(f"{where}: no `pip3 check` after the install")
    assert not problems, "; ".join(problems)


def test_check_1340_requirement_files_are_exact_unique_pins() -> None:
    """CHECK-1340: each requirements file the Dockerfile installs from is a
    COPY'd file under docker/, holds only `name==version` lines with unique
    names, and pins the four dev tools."""
    runs = _pip_runs()
    assert runs, "no pip install RUN found in docker/Dockerfile"
    copies = {
        Path(parts[2]).name: parts[1]
        for _, text in _instructions()
        if text.startswith("COPY ") and len(parts := text.split()) == 3
    }
    seen = 0
    for number, segments in runs:
        for segment in segments:
            if not _is_pip_install(segment):
                continue
            for target in _requirement_files(segment):
                where = f"docker/Dockerfile:{number}"
                seen += 1
                source = copies.get(Path(target).name)
                assert source, f"{where}: {target} is not COPY'd in"
                path = REPO_ROOT / source
                assert path.is_file(), f"{where}: {source} does not exist"
                names: list[str] = []
                for line in path.read_text().splitlines():
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    assert PIN_LINE.match(line), (
                        f"{source}: not name==version: {line!r}"
                    )
                    names.append(
                        re.sub(r"[-_.]+", "-", line.split("==")[0]).lower()
                    )
                assert len(names) == len(set(names)), (
                    f"{source}: duplicate names"
                )
                assert set(names) >= TOOLS, (
                    f"{source}: missing {TOOLS - set(names)}"
                )
    assert seen, "pip install reads no -r requirements file"
