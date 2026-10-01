---
type: codebase
description: Sphinx extension that wraps every HTML image in a Spotlight.js lightbox link, with SVG-safe sizing and a no-spotlight opt-out.
source: py_packages/sphinxcontrib_spotlight
source_digest: sha256:137cc860d75d3181fb91833eb7803d470db3e057bd7ee4d3b9d6eec4148e1355
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: code
  resource: py_packages/sphinxcontrib_spotlight
---

# sphinxcontrib_spotlight

"A Sphinx extension that integrates Spotlight.js for image lightbox support in
documentation." It is the `sphinxcontrib-spotlight` 0.1.0 distribution (MIT),
installed editable into `.venv` and enabled in `docs/source/conf.py`.

## Public surface

- The extension name `sphinxcontrib.spotlight`.
- The `no-spotlight` image class, which excludes an image.
- `data-*` attributes on the reference pass through as Spotlight options. The
  image alt text becomes `data-description`.

## How it works

1. `SpotlightTransform`, an HTML-only `SphinxPostTransform` at priority 200,
   replaces each image node with a `spotlight_reference` node around the image.
   It skips images that are already linked, opted out, or have no URI.
2. At write time, `visit_spotlight_reference` resolves the image's output path
   under `_images/` and emits `<a class="spotlight" href=…>`.
3. `setup` registers the node and the transform and adds `_static/`, which holds
   the bundled `spotlight.bundle.min.js`.

## Depends on

- `sphinx>=5.0`, `docutils`
- Bundled Spotlight.js 0.7.8 (Apache-2.0, see `THIRD_PARTY_LICENSES.md`)

## Invariants & gotchas

- The `href` is resolved at write time, not in the transform, because only then
  does the builder know the copied image path. Resolving it earlier links to the
  source path.
- `sphinxcontrib/__init__.py` is a `pkgutil` namespace package, so it can
  coexist with other `sphinxcontrib.*` extensions.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `py_packages/sphinxcontrib_spotlight/sphinxcontrib/spotlight/__init__.py` —
  transform, node visitor, `setup`
- `py_packages/sphinxcontrib_spotlight/pyproject.toml` — distribution metadata
- `docs/source/conf.py:68` — extension enabled
