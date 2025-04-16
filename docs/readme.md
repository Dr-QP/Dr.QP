# Sphinx documentation

This directory contains the Sphinx documentation for the project.

## Building the documentation

```bash
python -m venv .venv-docs
.venv-docs/bin/python -m pip install -r docs/requirements.txt

.venv-docs/bin/sphinx-build -nW --keep-going -b html docs/source/ docs/_build/html
```
