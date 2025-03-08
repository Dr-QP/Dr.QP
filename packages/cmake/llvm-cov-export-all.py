#!/usr/bin/env python3
#!/usr/bin/env python3

from pathlib import Path

import argparse
import subprocess

is_verbose = False

def verbose(*args_, **kwargs):
    if is_verbose:
        print(args_, **kwargs)

def process(binary, profile, output):
    merged_profile = Path(profile).with_suffix(".profdata")
    verbose(f"Merging profile {profile} into {merged_profile}")
    subprocess.run(["llvm-profdata", "merge", "-sparse", profile, "-o", str(merged_profile)])

    verbose(f"Exporting coverage for {binary} with profile {profile} to {output}")
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["llvm-cov", "export", "--format", "lcov", binary, "-instr-profile", merged_profile], stdout=open(output, "w"))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process llvm source based coverage profiles and export as lcov format")
    parser.add_argument("base_path", help="The base path to search for binaries and profiles")
    args = parser.parse_args()

    for profile in Path(args.base_path).rglob("*.profraw"):
        print(f"Processing raw profile {profile}")
        binary = profile.with_suffix("")
        output = binary.with_suffix(".lcov.info")
        process(binary, profile, output)
