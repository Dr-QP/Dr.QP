#!/usr/bin/env python3

"""Compatibility wrapper for validate_agents.validators.agents."""

from ...validators.agents import (
    build_known_agent_targets,
    validate_agent_frontmatter,
    validate_handoff,
)

__all__ = ['build_known_agent_targets', 'validate_agent_frontmatter', 'validate_handoff']
