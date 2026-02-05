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

"""Test edge cases and boundary conditions."""

import sys
from pathlib import Path

import pytest

# Import module under test
sys.path.insert(0, str(Path(__file__).parent.parent.parent))
validate_skills = __import__('validate-skills')


class TestEdgeCases:
    """Test edge cases and boundary conditions."""

    def test_empty_skill_file(self, tmp_path: Path):
        """Should handle completely empty skill file."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text('')

        all_skills = validate_skills.load_all_skills([str(skill_file)])
        result = validate_skills.validate_skill(
            str(skill_file), show_warnings=False, all_skills=all_skills, repo_root=str(tmp_path)
        )

        assert len(result.issues) > 0

    def test_only_frontmatter_no_body(self, tmp_path: Path):
        """Should handle skill with only frontmatter, no body."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: empty-body
description: Skill with no body content
license: MIT
---
""")

        all_skills = validate_skills.load_all_skills([str(skill_file)])
        result = validate_skills.validate_skill(
            str(skill_file), show_warnings=False, all_skills=all_skills, repo_root=str(tmp_path)
        )

        assert len(result.issues) > 0

    def test_special_characters_in_paths(self, tmp_path: Path):
        """Should handle paths with special characters."""
        special_dir = tmp_path / 'path with spaces' / 'special-chars!'
        special_dir.mkdir(parents=True)
        skill_file = special_dir / 'SKILL.md'
        skill_file.write_text("""---
name: special-path
description: Skill in path with special characters
license: MIT
---
# Test
""")

        result = validate_skills.find_skill_files(str(special_dir))
        assert len(result) == 1

    def test_very_long_skill_content(self, tmp_path: Path):
        """Should handle skills with very large content."""
        skill_file = tmp_path / 'SKILL.md'
        large_content = 'x' * 100000  # 100KB of content
        skill_file.write_text(f"""---
name: large-skill
description: Skill with very large content body
license: MIT
---
# Large Skill

{large_content}
""")

        all_skills = validate_skills.load_all_skills([str(skill_file)])
        assert str(skill_file) in all_skills

    def test_unicode_content(self, tmp_path: Path):
        """Should handle Unicode characters in skill content."""
        skill_file = tmp_path / 'SKILL.md'
        skill_file.write_text("""---
name: unicode-skill
description: "Skill with Unicode: 你好世界 🚀 émojis and açcênts"
license: MIT
---
# Unicode Skill 🎉

Content with various Unicode: αβγδ, 한글, العربية
""")

        all_skills = validate_skills.load_all_skills([str(skill_file)])
        assert str(skill_file) in all_skills

    def test_symlink_handling(self, tmp_path: Path):
        """Should handle symlinked skill files appropriately."""
        real_skill = tmp_path / 'real' / 'SKILL.md'
        real_skill.parent.mkdir()
        real_skill.write_text("""---
name: real-skill
description: Real skill file
license: MIT
---
# Real
""")

        link_dir = tmp_path / 'link'
        link_dir.mkdir()
        link_skill = link_dir / 'SKILL.md'

        try:
            link_skill.symlink_to(real_skill)
            result = validate_skills.find_skill_files(str(link_dir))
            assert len(result) == 1
        except OSError:
            pytest.skip('Symlinks not supported on this system')
