"""Tests for the ROS-packaged drqp_robot_mcp migration."""

from __future__ import annotations

from pathlib import Path
import unittest

import pytest


def _run_pytest_suite() -> None:
    test_dir = Path(__file__).parent
    exit_code = pytest.main([str(test_dir), '-q'])
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
