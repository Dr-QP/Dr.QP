#!/usr/bin/env python3
# Copyright (c) Anton Matosov
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import os
import sys
import subprocess
import yaml


def get_vendor_package_source_info(package_path):
    source_info_path = os.path.join(package_path, 'source-info.yaml')
    if not os.path.isfile(source_info_path):
        print(f'No source-info.yaml found in {package_path}, skipping.')
        return None
    with open(source_info_path, 'r') as f:
        source_info = yaml.safe_load(f)
    repo_url = source_info.get('repo')
    if not repo_url:
        print(f'No repo URL found in {source_info_path}, skipping.')
        return None
    branch = source_info.get('branch')
    if not branch:
        print(f'No branch specified in {source_info_path}, skipping.')
        return None
    return source_info


def set_vendor_package_source_info(package_path, source_info):
    source_info_path = os.path.join(package_path, 'source-info.yaml')
    with open(source_info_path, 'w') as f:
        yaml.dump(source_info, f)
    print(f'Updated {source_info_path} with new source info.')


def update_vendor_package(package_path):
    """
    Update a single vendor package located at package_path.

    Use git subtree to pull in the latest changes from the specified
    repository, branch.
    Update revision in the source-info.yaml file accordingly.
    """
    if not os.path.isdir(package_path):
        print(f'Package path {package_path} does not exist, skipping.')
        return
    source_info = get_vendor_package_source_info(package_path)
    if not source_info:
        return
    repo_url = source_info.get('repo')
    branch = source_info.get('branch')
    print(f'Updating package at {package_path} from {repo_url} (branch: {branch})')
    subprocess.run(
        ['git', 'subtree', 'pull', '--prefix', package_path, repo_url, branch, '--squash'],
        check=True,
    )
    # Get the new revision (latest commit hash) from the remote repository
    source_info['rev'] = new_rev

    set_vendor_package_source_info(package_path, source_info)
