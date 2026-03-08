#!/usr/bin/env python3

"""Tests for the canonical validate_agent_files CLI parser."""

from __future__ import annotations

import pytest

from validate_agent_files.cli import parse_arguments


def test_parse_arguments_defaults_to_all_kinds() -> None:
    """The canonical parser should validate all customization kinds by default."""
    parsed = parse_arguments([])

    assert parsed.path == '.'
    assert parsed.kind == 'all'
    assert parsed.format == 'text'


def test_parse_arguments_supports_skills_filter() -> None:
    """The canonical parser should allow skills-only validation."""
    parsed = parse_arguments(['--kind', 'skills', '.github/skills'])

    assert parsed.kind == 'skills'
    assert parsed.path == '.github/skills'


def test_parse_arguments_rejects_xml_output() -> None:
    """The canonical parser should reject the removed xml output format."""
    with pytest.raises(SystemExit):
        parse_arguments(['--format', 'xml'])
