# sphinxcontrib-glightbox

A Sphinx extension that integrates [GLightbox](https://github.com/biati-digital/glightbox) for better image lightbox support in documentation.

## Features

- Properly handles SVG images (unlike lightbox2 which shows them oversized)
- Pure JavaScript implementation with mobile support
- Touch navigation support
- Zoom support

## Usage

Add `sphinxcontrib.glightbox` to your Sphinx extensions in `conf.py`:

```python
extensions = [
    ...
    'sphinxcontrib.glightbox',
    ...
]
```

All images in the documentation will automatically be wrapped with GLightbox links.

To exclude a specific image from the lightbox, add the `no-glightbox` class:

```rst
.. image:: image.png
   :class: no-glightbox
```

## Third-Party Licenses

This package bundles GLightbox CSS and JavaScript assets. See [THIRD_PARTY_LICENSES.md](./THIRD_PARTY_LICENSES.md) for the full license text.
