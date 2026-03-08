#!/usr/bin/env python3

"""Tests for repo-specific agent file validation."""

from __future__ import annotations

from pathlib import Path

from validate_agents.main import main


def test_issue310_agent_validation_requires_repo_frontmatter_and_handoff_shape(
    tmp_path: Path, capsys
) -> None:
    """Agent files should enforce required repo schema fields."""
    valid_agent = tmp_path / 'valid.agent.md'
    valid_agent.write_text(
        """---
name: Valid Agent
description: Valid description.
model: GPT-5.4
tools: [read]
handoffs:
  - label: Next
    agent: other-agent
---

# Valid Agent
"""
    )

    missing_description = tmp_path / 'missing-description.agent.md'
    missing_description.write_text(
        """---
name: Missing Description
model: GPT-5.4
tools: [read]
---

# Missing Description
"""
    )

    bad_handoff = tmp_path / 'bad-handoff.agent.md'
    bad_handoff.write_text(
        """---
name: Bad Handoff
description: Valid description.
model: GPT-5.4
tools: [read]
handoffs:
  - label: Missing target
---

# Bad Handoff
"""
    )

    exit_code = main([str(tmp_path), '--kind', 'agents'])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert 'missing-description.agent.md' in captured.out
    assert 'description' in captured.out
    assert 'bad-handoff.agent.md' in captured.out
    assert 'handoffs' in captured.out


def test_issue310_agent_validation_checks_handoff_target_exists(
    tmp_path: Path, capsys
) -> None:
    """Handoffs should resolve to an existing agent by file stem or name."""
    (tmp_path / 'existing.agent.md').write_text(
        """---
name: Existing Agent
description: Valid description.
model: GPT-5.4
tools: [read]
---

# Existing Agent
"""
    )
    (tmp_path / 'missing-target.agent.md').write_text(
        """---
name: Missing Target
description: Valid description.
model: GPT-5.4
tools: [read]
handoffs:
  - label: Go missing
    agent: does-not-exist
---

# Missing Target
"""
    )

    exit_code = main([str(tmp_path), '--kind', 'agents'])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert 'does-not-exist' in captured.out
