# Sphinx documentation

This directory contains the Sphinx documentation for the project.

## Building the documentation

### TL;DR

 1. Clone repo and open workspace as usual.
 2. Run `Dr.QP venv` task from VSCode
 3. Run `Dr.QP build docs` task from VSCode

### Details

The following commands will create a clean venv specific for building documentation and build the documentation using Sphinx.

```bash
python -m venv .venv-docs
.venv-docs/bin/python -m pip install -r docs/requirements.txt

.venv-docs/bin/sphinx-build -nW --keep-going -b html docs/source/ docs/_build/html
```
