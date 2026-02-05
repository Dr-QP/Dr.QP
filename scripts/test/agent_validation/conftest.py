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

"""Shared fixtures for Agent Skill validation tests."""

import sys
from pathlib import Path
from typing import Dict

import pytest


# Add scripts directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))


@pytest.fixture
def good_skill_path(tmp_path: Path) -> Path:
    """Return path to a valid skill file."""
    skill_dir = tmp_path / 'good-skill'
    skill_dir.mkdir()
    skill_file = skill_dir / 'SKILL.md'
    skill_file.write_text("""---
name: example-good-skill
description: Toolkit for testing local web applications using Playwright browser automation. Use when asked to verify frontend functionality, debug UI behavior, capture browser screenshots, check for visual regressions, or view browser console logs. Supports Chrome, Firefox, and WebKit browsers for comprehensive cross-browser testing.
license: Complete terms in LICENSE.txt
---

# Example Good Skill

This skill demonstrates proper structure and best practices for Agent Skills.

## When to Use This Skill

Use this skill when you need to:

- Test web application functionality in a real browser
- Capture screenshots for visual regression testing
- Debug UI issues by inspecting browser console logs
- Verify cross-browser compatibility
- Automate user interactions for testing

## Prerequisites

Before using this skill, ensure you have:

- Node.js 18+ installed
- Playwright package installed (`npm install -D @playwright/test`)
- A local web server running (e.g., `npm run dev`)
- Access to the application under test

## Testing Workflow

### 1. Setup Test Environment

```bash
# Install Playwright browsers
npx playwright install
```

### 2. Run Tests

```bash
# Run all tests
npx playwright test

# Run specific test
npx playwright test example.spec.ts
```

## Debugging Workflow

### 1. Capture Screenshots

Use the screenshot tool to capture current page state:

```bash
npx playwright test --headed --debug
```

### 2. Inspect Console Logs

Review browser console for JavaScript errors and warnings.

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Browser not launching | Run `npx playwright install` to install browsers |
| Tests timing out | Increase timeout in playwright.config.ts |
| Flaky tests | Add explicit waits for elements |
| Screenshots differ | Update baseline screenshots |

## References

- [Playwright Documentation](https://playwright.dev)
- [Best Practices Guide](https://playwright.dev/docs/best-practices)
""")
    return skill_file


@pytest.fixture
def bad_skill_path(tmp_path: Path) -> Path:
    """Return path to an invalid skill file."""
    skill_dir = tmp_path / 'bad-skill'
    skill_dir.mkdir()
    skill_file = skill_dir / 'SKILL.md'
    skill_file.write_text("""---
name: Bad_Skill_Name
description: Too short
---

# Bad Skill Example

This is a bad skill that violates multiple validation rules.

## Some Section

This skill is missing required sections and has various problems.
""")
    return skill_file


@pytest.fixture
def temp_skill_dir(tmp_path: Path) -> Path:
    """Create a temporary directory for test skills."""
    skill_dir = tmp_path / 'skills'
    skill_dir.mkdir()
    return skill_dir


@pytest.fixture
def sample_frontmatter() -> Dict:
    """Return sample valid frontmatter."""
    return {'name': 'test-skill', 'description': 'A test skill for validation', 'license': 'MIT'}


@pytest.fixture
def sample_body() -> str:
    """Return sample valid skill body."""
    return """# Test Skill

## When to Use This Skill

Use this when testing.

## Prerequisites

- Python 3.8+
- pytest
"""
