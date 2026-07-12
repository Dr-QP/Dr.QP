#!/usr/bin/env python3

"""Regression checks for the retargeted TDD agents (spec 02, AC1/AC3)."""

from __future__ import annotations

from pathlib import Path
import re

REPO_ROOT = Path(__file__).resolve().parents[3]
AGENTS_DIR = REPO_ROOT / '.claude' / 'agents'

CSHARP_TOKENS = re.compile(
    r'xUnit|FluentAssertions|AutoFixture|NuGet|Serilog|Key Vault'
    r'|IOptions|Span<T>|SonarQube|Checkmarx'
)
CONFIRM_WITH_USER = re.compile(r'[Cc]onfirm .* with the user')


def test_no_csharp_tooling_in_agent_files() -> None:
    """No .NET/C# stack references survive under .claude/agents/."""
    offenders = [
        path.name
        for path in AGENTS_DIR.glob('*.agent.md')
        if CSHARP_TOKENS.search(path.read_text(encoding='utf-8'))
    ]
    assert offenders == [], f'C#/.NET tokens found in: {offenders}'


def test_no_confirm_with_user_in_tdd_agents() -> None:
    """TDD agents never block on user confirmation (sub-agents can't reach one)."""
    offenders = [
        path.name
        for path in AGENTS_DIR.glob('tdd-*.agent.md')
        if CONFIRM_WITH_USER.search(path.read_text(encoding='utf-8'))
    ]
    assert offenders == [], f'"confirm with the user" found in: {offenders}'


def test_tdd_agents_do_not_grant_agent_tool() -> None:
    """Leaf TDD workers drop the Agent tool (AC12, least privilege)."""
    offenders = []
    for path in AGENTS_DIR.glob('tdd-*.agent.md'):
        tools_line = next(
            (
                line
                for line in path.read_text(encoding='utf-8').splitlines()
                if line.startswith('tools:')
            ),
            '',
        )
        if 'Agent' in tools_line:
            offenders.append(path.name)
    assert offenders == [], f'Agent tool granted in: {offenders}'
