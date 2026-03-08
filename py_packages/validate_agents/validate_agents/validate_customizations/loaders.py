#!/usr/bin/env python3

"""Compatibility wrapper for validate_agents.loaders."""

from ..loaders import (
    CustomFileContent,
    agent_identifier,
    find_agent_files,
    find_prompt_files,
    load_custom_file,
)

__all__ = [
    'CustomFileContent',
    'agent_identifier',
    'find_agent_files',
    'find_prompt_files',
    'load_custom_file',
]
