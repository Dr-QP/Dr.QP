# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pytest

pytest_plugins = ['pytester']


@pytest.fixture
def testdir(testdir):
    """
    Disable pytest-retry inside nested pytester runs.

    pytest-retry (>=1.7.0) keeps a process-global ``Defaults`` singleton that its
    ``pytest_configure`` mutates by appending ``FILTERED_EXCEPTIONS`` /
    ``EXCLUDED_EXCEPTIONS`` entries. Because ``testdir.runpytest()`` executes
    in-process, every nested run reuses that already-mutated singleton, and
    ``Defaults.configure`` then calls ``config.getoption('filtered_exceptions')``
    for an option that was never registered as a CLI flag -- raising an
    INTERNALERROR that fails these self-tests. The plugin is irrelevant to
    launch_pytest's own suite, so switch it off for the nested runs.
    """
    original_runpytest = testdir.runpytest

    def runpytest(*args, **kwargs):
        return original_runpytest('-p', 'no:pytest-retry', *args, **kwargs)

    testdir.runpytest = runpytest
    return testdir
