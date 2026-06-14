#!/usr/bin/env python3
"""Install generated overlay Python requirements in one pip invocation."""

from __future__ import annotations

import os
import re
import subprocess
import sys
from collections.abc import Iterator
from pathlib import Path


SKIP_REQUIREMENT_RE = re.compile(r"^\s*(#|\[|$)")


def iter_requires_files(search_roots: list[Path]) -> Iterator[Path]:
    for root in search_roots:
        if not root.is_dir():
            continue
        yield from root.rglob("requires.txt")


def collect_requirements(requires_files: list[Path]) -> tuple[list[str], list[Path]]:
    requirements: list[str] = []
    contributing_files: list[Path] = []

    for path in requires_files:
        path_requirements: list[str] = []
        with path.open("r", encoding="utf-8") as requires_file:
            for line in requires_file:
                if SKIP_REQUIREMENT_RE.match(line):
                    continue
                path_requirements.append(line.rstrip("\n"))

        if path_requirements:
            contributing_files.append(path)
            requirements.extend(path_requirements)

    return requirements, contributing_files


def build_pip_command(constraints_file: Path, requirements: list[str]) -> list[str]:
    command = [
        "/usr/bin/python3",
        "-m",
        "pip",
        "install",
        "--break-system-packages",
        "--ignore-installed",
    ]

    if constraints_file.is_file():
        command.extend(["-c", str(constraints_file)])

    command.extend(requirements)

    if os.geteuid() != 0:
        command.insert(0, "sudo")

    return command


def main() -> int:
    search_roots = [Path(arg) for arg in sys.argv[1:]]
    if not search_roots:
        search_roots = [Path("build"), Path("install")]

    requires_files = sorted(iter_requires_files(search_roots))
    requirements, contributing_files = collect_requirements(requires_files)
    if not requirements:
        return 0

    script_dir = Path(__file__).resolve().parent
    constraints_file = script_dir / "pip-constraints.txt"

    print("Installing requirements from:")
    for path in contributing_files:
        print(f"  {path}")

    subprocess.run(
        build_pip_command(constraints_file, requirements),
        check=True,
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
