---
description: 'Extra Python guidelines for local scripts, docs and notebooks'
applyTo: 'docs/**/*.py, docs/**/notebooks/*.md, scripts/**/*.py'
---

# Python Coding Conventions for Local Scripts and Documentation

These guidelines supplement the [general Python coding conventions](./python.instructions.md) with practices tailored for local scripts, documentation examples, and Jupyter notebooks.

## Execution Environment

- Use virtual environment `.venv` for executing scripts locally
- Define workspace dependencies in `pyproject.toml` and sync `.venv` with `uv`
- Avoid using global installations of packages to prevent version conflicts

## Documentation Examples

- Ensure code snippets in documentation are executable and tested
- Use inline comments to explain complex logic within code examples
- Prefer using simple and clear examples that illustrate the concept effectively

## Testing

- Always use `pytest` for writing tests and executing them in local environments
