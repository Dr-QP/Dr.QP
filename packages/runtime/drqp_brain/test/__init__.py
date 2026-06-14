# Copyright (c) 2017-2026 Anton Matosov
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

"""Test loader for drqp_brain package tests."""

from __future__ import annotations

from pathlib import Path
import unittest
import warnings

import pytest
from pytest import PytestDeprecationWarning


def _run_pytest_suite() -> None:
    test_dir = Path(__file__).parent
    with warnings.catch_warnings():
        warnings.filterwarnings(
            'ignore',
            message=r'The \(path: py\.path\.local\) argument is deprecated.*',
            category=PytestDeprecationWarning,
        )
        exit_code = pytest.main([test_dir, '-q'])
    if exit_code != 0:
        raise AssertionError(f'pytest exited with code {exit_code}')


def load_tests(
    loader: unittest.TestLoader,
    tests: unittest.TestSuite,
    pattern: str | None,
) -> unittest.TestSuite:
    del loader, tests, pattern

    suite = unittest.TestSuite()
    suite.addTest(unittest.FunctionTestCase(_run_pytest_suite))
    return suite
