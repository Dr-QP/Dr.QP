#!/usr/bin/env python3

"""Compatibility wrapper for validate_agents.validators.prompts."""

from ...validators.prompts import (
    validate_prompt_body,
    validate_prompt_frontmatter,
    validate_prompt_references,
)

__all__ = [
    'validate_prompt_body',
    'validate_prompt_frontmatter',
    'validate_prompt_references',
]
