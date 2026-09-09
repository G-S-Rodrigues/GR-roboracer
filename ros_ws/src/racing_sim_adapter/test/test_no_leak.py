"""Structural checks for the backend-neutral simulator adapter contract."""

from pathlib import Path


def test_adapt_2060_adapter_headers_do_not_leak_gym_jax() -> None:
    """ADAPT-2060: adapter headers contain no f1tenth_gym_jax symbol."""
    include_dir = Path(__file__).parents[1] / "include"
    headers = sorted(include_dir.rglob("*.h")) + sorted(
        include_dir.rglob("*.hpp")
    )

    assert headers, "adapter contract must install at least one header"
    leaked = [path for path in headers if "f1tenth_gym_jax" in path.read_text()]
    assert leaked == []
