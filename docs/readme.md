# Sphinx documentation

This directory contains the Sphinx documentation for the project.

## Building the documentation

### TL;DR

1.  Clone repo and open workspace as usual.
2.  Run `Dr.QP venv` task from VSCode (`uv sync`)
3.  Run `Dr.QP build docs` task from VSCode

### Details

The following commands will set up the workspace virtual environment and build the documentation using Sphinx.

```bash
python3 -m pip install --user --break-system-packages --disable-pip-version-check uv
$HOME/.local/bin/uv sync

.venv/bin/sphinx-build -nW --keep-going -b html docs/source/ docs/_build/html
```
