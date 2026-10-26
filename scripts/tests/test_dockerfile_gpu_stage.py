"""Structural guards on the `gpu` image stage (issue #47 part 3).

The stage is built and run only on the owner's machine (no GPU runner), so CI
holds its shape: the stage graph, the apt closure pin and the compose service.
GPU-3010 (docker/gpu/smoke.sh) is the manual half.
"""

from __future__ import annotations

import re
import shlex
from pathlib import Path

import yaml

from scripts.dockerfile_parse import REPO_ROOT, segments, stages

PINS = REPO_ROOT / "docker" / "gpu-apt-pins.txt"
COMPOSE = REPO_ROOT / "docker" / "docker-compose.yaml"
KEYRING_URL = (
    "https://developer.download.nvidia.com/compute/cuda/repos/"
    "ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb"
)
PIN_LINE = re.compile(r"[a-z0-9][a-z0-9.+-]*=[0-9][0-9A-Za-z.+~-]*")
ORDER = ["base", "underlay", "dev", "gpu", "fast-sim", "runtime"]


def _code(stage_name: str) -> list[tuple[int, str]]:
    """A stage's instructions with comment-only lines dropped."""
    return [
        (n, t)
        for n, t in stages()[stage_name].instructions
        if not t.lstrip().startswith("#")
    ]


def _gpu_runs() -> list[tuple[int, str]]:
    assert "gpu" in stages(), "no gpu stage"
    return [(n, t) for n, t in _code("gpu") if t.startswith("RUN ")]


def test_check_1900_stage_graph():
    found = stages()
    assert "gpu" in found, "no gpu stage"
    assert found["gpu"].parent == "dev", "gpu must be FROM dev"
    assert list(found) == ORDER, f"stage order is {list(found)}"
    assert not [n for n, s in found.items() if s.parent == "gpu"], (
        "no stage may derive from gpu"
    )
    for name, stage in found.items():
        if name == "gpu":
            continue
        for _, text in stage.instructions:
            assert not re.search(r"--from=gpu\b", text), (
                f"{name} copies from the gpu stage: {text[:80]}"
            )
    for name in ("dev", "underlay", "base"):
        for _, text in _code(name):
            assert not re.search(r"cuda|nvidia", text, re.I), (
                f"{name} stage carries CUDA: {text[:80]}"
            )
    for _, text in _code("fast-sim"):
        assert not re.search(r"\bros|jazzy", text, re.I), (
            f"fast-sim carries ROS: {text[:80]}"
        )


def _is_apt_install(seg: list[str]) -> bool:
    for i, tok in enumerate(seg):
        if tok in ("apt-get", "apt") and "install" in seg[i + 1 :]:
            return True
    return False


def test_check_1910_install_shape():
    code = _code("gpu") if "gpu" in stages() else []
    assert code, "no gpu stage"
    copies = [t.split() for _, t in code if t.startswith("COPY ")]
    pins = [c for c in copies if c[1:2] == ["docker/gpu-apt-pins.txt"]]
    assert len(pins) == 1, "pin file is not COPY'd exactly once"
    pins_dest = pins[0][-1]

    installs = []
    for number, text in _gpu_runs():
        assert not [
            t for t in shlex.split(text.removeprefix("RUN")) if t[:1] == "#"
        ], f"comment inside the RUN at line {number}"
        for seg in segments(text):
            if _is_apt_install(seg):
                installs.append((number, text, seg))
    assert len(installs) == 1, f"want one apt install, got {len(installs)}"
    _, text, seg = installs[0]
    assert seg[:2] == ["xargs", "-a"], "install is not `xargs -a <list> ...`"
    list_file = seg[2]
    assert seg[3:5] == ["apt-get", "install"], f"xargs runs {seg[3:5]}"
    assert "--no-install-recommends" in seg and "-y" in seg
    assert "||" not in shlex.split(text.removeprefix("RUN")), (
        "a `||` can swallow a failing step"
    )

    made = re.search(
        rf"grep -v '\^#' {re.escape(pins_dest)}[^&;]*>\s*(\S+)", text
    )
    assert made, f"list is not built from the COPY'd {pins_dest}"
    assert made.group(1) == list_file, (
        f"xargs reads {list_file}, not the list built from {pins_dest}"
    )
    queried = re.search(r"dpkg-query[^&;]*'cuda-\*'[^&;]*>\s*(\S+)", text)
    assert queried, "no dpkg-query over cuda-* written to a file"
    diffs = [s for s in segments(text) if s and s[0] == "diff"]
    assert len(diffs) == 1, "want exactly one diff"
    assert diffs[0][1:] == [list_file, queried.group(1)], (
        f"diff compares {diffs[0][1:]}, not the pin list and the installed list"
    )
    after = text.index("xargs -a")
    assert text.index("diff ", after) > text.index("dpkg-query", after)
    for _, t in code:
        assert not re.search(r"\b(pip3?|uv)\b.*\binstall\b|\buv\s+pip\b", t), (
            f"gpu installs Python packages: {t[:80]}"
        )


def test_check_1920_pin_file():
    assert PINS.is_file(), "docker/gpu-apt-pins.txt missing"
    lines = [
        line.strip()
        for line in PINS.read_text().splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    names = []
    for line in lines:
        assert PIN_LINE.fullmatch(line), f"not a name=version line: {line!r}"
        name, version = line.split("=")
        assert name.startswith("cuda-"), f"not a cuda package: {name}"
        if name != "cuda-keyring":
            assert version.startswith("12.8."), f"{name} is {version}"
        names.append(name)
    assert len(names) == len(set(names)), "duplicate package"
    assert "cuda-nvcc-12-8" in names and "cuda-cudart-12-8" in names


def test_check_1930_keyring_checksum():
    runs = _gpu_runs()
    keyring = [(n, t) for n, t in runs if KEYRING_URL in t]
    assert keyring, "keyring URL not fetched by exact URL"
    text = keyring[0][1]
    assert not re.search(r"curl[^|]*\|\s*(sh|bash)\b", text), "curl | sh in gpu"
    assert "sha256sum -c" in text and "dpkg -i" in text
    assert text.index("sha256sum -c") < text.index("dpkg -i"), (
        "keyring is installed before it is checked"
    )
    arg = re.search(r"ARG CUDA_KEYRING_SHA256=([0-9a-f]{64})", _header())
    assert arg, "no pinned ARG CUDA_KEYRING_SHA256 at top level"
    assert "ARG CUDA_KEYRING_SHA256" in [t.strip() for _, t in _code("gpu")], (
        "ARG not redeclared inside gpu"
    )
    assert "${CUDA_KEYRING_SHA256}" in text


def _header() -> str:
    return (REPO_ROOT / "docker" / "Dockerfile").read_text()


def test_check_1940_compose_service():
    compose = yaml.safe_load(Path(COMPOSE).read_text())
    services = compose["services"]
    assert "gpu" in services, "no gpu service"
    gpu, dev = services["gpu"], services["dev"]
    assert gpu.get("profiles") == ["gpu"], "gpu service must be profile gpu"
    assert gpu["build"]["target"] == "gpu"
    devices = gpu["deploy"]["resources"]["reservations"]["devices"]
    assert any(d.get("driver") == "nvidia" for d in devices)
    assert "deploy" not in dev, "dev must not reserve a GPU"

    def mounts(svc):
        out = {}
        for v in svc["volumes"]:
            src, dst = v.split(":")[:2]
            out[dst] = src
        return out

    for vol, body in compose["volumes"].items():
        assert not (body or {}).get("name"), f"{vol} sets name: (shared volume)"
    gm, dm = mounts(gpu), mounts(dev)
    for path in ("/ws/build", "/ws/install", "/ws/log"):
        assert path in gm, f"gpu does not mount {path}"
        assert gm[path] != dm[path], f"gpu shares dev's volume at {path}"
        assert gm[path] in compose["volumes"], f"{gm[path]} not declared"
    assert (
        gpu["environment"]["ROS_DOMAIN_ID"]
        != dev["environment"]["ROS_DOMAIN_ID"]
    )
