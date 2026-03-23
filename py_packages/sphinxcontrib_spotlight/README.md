# sphinxcontrib-spotlight

A Sphinx extension that integrates [Spotlight.js](https://github.com/nextapps-de/spotlight) for image lightbox support in documentation.

## Features

- Properly handles SVG images
- Pure JavaScript implementation with no dependencies
- Touch, keyboard, and mouse controls
- Mobile-responsive

## Usage

Add `sphinxcontrib.spotlight` to your Sphinx extensions in `conf.py`:

```python
extensions = [
    ...
    'sphinxcontrib.spotlight',
    ...
]
```

All images in the documentation will automatically be wrapped with Spotlight links.

To exclude a specific image from the lightbox, add the `no-spotlight` class:

```rst
.. image:: image.png
   :class: no-spotlight
```

## Third-Party Licenses

This package bundles Spotlight.js assets. See [THIRD_PARTY_LICENSES.md](./THIRD_PARTY_LICENSES.md) for the full license text.
