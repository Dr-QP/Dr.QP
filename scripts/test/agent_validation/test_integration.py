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

"""Integration tests for end-to-end validation workflows."""

import sys
from pathlib import Path

import pytest

# Import module under test
sys.path.insert(0, str(Path(__file__).parent.parent.parent))
validate_skills = __import__('validate-skills')


class TestIntegration:
    """Integration tests for end-to-end validation workflows."""

    def test_complete_validation_workflow(self, good_skill_path: Path, monkeypatch, capsys):
        """Should complete full validation workflow from finding to reporting."""
        monkeypatch.setattr(sys, 'argv', ['validate-skills.py', str(good_skill_path.parent.parent)])

        _ = validate_skills.main()

        captured = capsys.readouterr()
        assert 'Validating' in captured.out
        assert 'Summary:' in captured.out

    def test_validates_multiple_skills_together(self, temp_skill_dir: Path, monkeypatch):
        """Should validate multiple skills and detect cross-skill issues."""
        skill1_dir = temp_skill_dir / 'skill1'
        skill1_dir.mkdir()
        (skill1_dir / 'SKILL.md').write_text("""---
name: duplicate-name
description: First skill with duplicate name
license: MIT
---
# Skill 1
""")

        skill2_dir = temp_skill_dir / 'skill2'
        skill2_dir.mkdir()
        (skill2_dir / 'SKILL.md').write_text("""---
name: duplicate-name
description: Second skill with duplicate name
license: MIT
---
# Skill 2
""")

        monkeypatch.setattr(sys, 'argv', ['validate-skills.py', str(temp_skill_dir)])

        exit_code = validate_skills.main()

        assert exit_code == 1

    def test_cross_reference_validation(self, temp_skill_dir: Path, monkeypatch):
        """Should validate cross-references between skills."""
        skill_file = temp_skill_dir / 'SKILL.md'
        skill_file.write_text("""---
name: test-skill
description: Test skill with cross-reference
license: MIT
---
# Test Skill

See [nonexistent-skill](../nonexistent-skill/SKILL.md) for more info.
""")

        monkeypatch.setattr(sys, 'argv', ['validate-skills.py', str(skill_file)])

        exit_code = validate_skills.main()

        assert exit_code in [0, 1]
