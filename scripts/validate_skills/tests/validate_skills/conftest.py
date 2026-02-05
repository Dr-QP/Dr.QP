#!/usr/bin/env python3
# Copyright (c) 2026 Dr.QP
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

"""Pytest configuration and shared fixtures."""

from pathlib import Path
import sys

import pytest

# Add parent directory to path for imports
test_dir = Path(__file__).parent
package_dir = test_dir.parent.parent
sys.path.insert(0, str(package_dir))


@pytest.fixture
def skill_template():
    """Provide a valid skill template for testing."""
    return """---
name: {name}
description: {description}
---
# Overview
This skill provides {name} functionality.

## When to use this skill
Use this skill when you need {name}.

## Features
- Feature 1
- Feature 2
"""


@pytest.fixture
def valid_skill_content():
    """Provide a complete valid skill content."""
    return """---
name: test-skill
description: A comprehensive description of what this skill does and when to use it.
---
# Overview
This skill is designed to validate other agent skills.

## When to use this skill
Use this skill when you need to validate Agent Skills and ensure they follow best practices.

## Features
- Validates frontmatter structure
- Checks file organization
- Detects duplicate skills
- Validates cross-references
"""


@pytest.fixture
def invalid_skill_content():
    """Provide invalid skill content for testing error handling."""
    return """---
description: Missing name field
---
Invalid content
"""


@pytest.fixture
def temp_skill_dir(tmp_path):
    """Create a temporary skill directory with sample skills."""
    skill1 = tmp_path / 'skill1' / 'SKILL.md'
    skill2 = tmp_path / 'skill2' / 'SKILL.md'

    skill1.parent.mkdir()
    skill2.parent.mkdir()

    skill1.write_text("""---
name: first-skill
description: A comprehensive description of the first test skill for validation.
---
# Overview
First skill overview.

## When to use this skill
Use when needed.
""")

    skill2.write_text("""---
name: second-skill
description: A comprehensive description of the second test skill for validation.
---
# Overview
Second skill overview.

## When to use this skill
Use when needed.
""")

    return tmp_path


@pytest.fixture
def mock_logger(monkeypatch):
    """Provide a mock logger for testing."""
    messages = []

    def log(msg):
        messages.append(msg)

    return log


# Hook to customize test output
def pytest_configure(config):
    """Configure pytest with custom settings."""
    config.addinivalue_line('markers', 'slow: mark test as slow to run')
    config.addinivalue_line('markers', 'integration: mark test as integration test')
