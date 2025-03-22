#!/usr/bin/env python3

import argparse
from pathlib import Path
import subprocess

is_verbose = False


def verbose(*args_, **kwargs):
    if is_verbose:
        print(args_, **kwargs)


def process(binary, profile, output):
    print(f'Exporting coverage to {output}')
    merged_profile = Path(profile).with_suffix('.profdata')
    subprocess.run(['llvm-profdata', 'merge', '-sparse', profile, '-o', str(merged_profile)])

    Path(output).parent.mkdir(parents=True, exist_ok=True)
    with open(output, 'w') as f:
        subprocess.run(
            ['llvm-cov', 'export', '--format', 'lcov', binary, '-instr-profile', merged_profile],
            stdout=f,
        )


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Process llvm source based coverage profiles and export as lcov format'
    )
    parser.add_argument('base_path', help='The base path to search for binaries and profiles')
    args = parser.parse_args()

    for profile in Path(args.base_path).rglob('*.profraw'):
        binary = profile.with_suffix('')
        output = binary.parent / 'coverage' / binary.name / 'lcov.info'
        process(binary, profile, output)
