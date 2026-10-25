"""Structural guards on the `gpu` image stage (issue #47 part 3).

The stage is built and run only on the owner's machine (no GPU runner), so CI
holds its shape: the stage graph, the apt closure pin and the compose service.
GPU-3010 (docker/gpu/smoke.sh) is the manual half.
"""

from __future__ import annotations

import re
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
    for name in ("dev", "underlay", "base"):
        for _, text in _code(name):
            assert not re.search(r"cuda|nvidia", text, re.I), (
                f"{name} stage carries CUDA: {text[:80]}"
            )
    for _, text in _code("fast-sim"):
        assert not re.search(r"\bros|jazzy", text, re.I), (
            f"fast-sim carries ROS: {text[:80]}"
        )


def test_check_1910_install_shape():
    runs = _gpu_runs()
    installs = [
        (n, t) for n, t in runs if "apt-get install" in t or "apt install" in t
    ]
    assert len(installs) == 1, f"want one apt install RUN, got {len(installs)}"
    number, text = installs[0]
    assert not re.search(r"(^|\n)\s*#", text), "comment inside the RUN"
    assert "COPY docker/gpu-apt-pins.txt" in " ".join(
        t for _, t in _code("gpu")
    ), "pin file is not COPY'd"
    segs = segments(text)
    xargs = [s for s in segs if s[:2] == ["xargs", "-a"]]
    assert len(xargs) == 1, "install is not `xargs -a <list> apt-get install`"
    tail = xargs[0][3:]
    assert tail[:2] == ["apt-get", "install"], f"xargs runs {tail[:2]}"
    assert "--no-install-recommends" in tail and "-y" in tail
    after = segs[segs.index(xargs[0]) + 1 :]
    query = [s for s in after if s and s[0] == "dpkg-query"]
    assert query and "cuda-*" in query[0], "no dpkg-query over cuda-*"
    assert re.search(r"dpkg-query[^|;&]*\|.*>\s*\S+", text), (
        "dpkg-query output is not written to a file"
    )
    diffs = [s for s in after if s and s[0] == "diff"]
    assert diffs and len(diffs[0]) == 3, "no diff of two files"
    for _, t in _code("gpu"):
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

    gm, dm = mounts(gpu), mounts(dev)
    for path in ("/ws/build", "/ws/install", "/ws/log"):
        assert path in gm, f"gpu does not mount {path}"
        assert gm[path] != dm[path], f"gpu shares dev's volume at {path}"
        assert gm[path] in compose["volumes"], f"{gm[path]} not declared"
    assert (
        gpu["environment"]["ROS_DOMAIN_ID"]
        != dev["environment"]["ROS_DOMAIN_ID"]
    )
