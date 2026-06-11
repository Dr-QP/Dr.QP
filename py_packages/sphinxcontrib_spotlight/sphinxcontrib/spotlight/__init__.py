"""
Sphinx extension for Spotlight.js - a pure JavaScript lightbox library.

This extension wraps images in the documentation with Spotlight links,
providing a lightweight lightbox experience. It properly handles SVG images
without rendering them oversized.
"""

from __future__ import annotations

from pathlib import Path
import posixpath
from typing import Any
import urllib.parse

from docutils import nodes
from sphinx.application import Sphinx
from sphinx.transforms.post_transforms import SphinxPostTransform
from sphinx.writers.html5 import HTML5Translator

_STATIC_DIR = Path(__file__).parent / '_static'


class spotlight_reference(nodes.reference):
    """
    Custom reference node for Spotlight-wrapped images.

    Using a custom node allows us to defer URI resolution to write time,
    so that the href correctly points to the output path in _images/ rather
    than the original source path.
    """


def visit_spotlight_reference(self: HTML5Translator, node: spotlight_reference) -> None:
    """Render a spotlight reference as an <a> tag with the correct image URI."""
    # Find the image child and resolve its output URI
    href = node.get('refuri', '')
    for child in node.children:
        if isinstance(child, nodes.image):
            old_uri = child['uri']
            if old_uri in self.builder.images:
                href = posixpath.join(
                    self.builder.imgpath,
                    urllib.parse.quote(self.builder.images[old_uri]),
                )
            else:
                href = old_uri
            break

    atts: dict[str, Any] = {'class': 'spotlight', 'href': href}

    # Pass through data-* attributes for Spotlight options
    for key, val in node.attributes.items():
        if key.startswith('data-'):
            atts[key] = val

    # Add alt text as spotlight description if available
    for child in node.children:
        if isinstance(child, nodes.image):
            alt = child.get('alt', '')
            if alt and 'data-description' not in atts:
                atts['data-description'] = alt
            break

    self.body.append(self.starttag(node, 'a', '', **atts))


def depart_spotlight_reference(self: HTML5Translator, node: spotlight_reference) -> None:
    """Close the <a> tag for the spotlight reference."""
    self.body.append('</a>')


class SpotlightTransform(SphinxPostTransform):
    """Post-transform that wraps image nodes with Spotlight anchor tags."""

    default_priority = 200
    formats = ('html',)

    def run(self, **kwargs: Any) -> None:
        for node in self.document.findall(nodes.image):
            self._wrap_image(node)

    def _wrap_image(self, node: nodes.image) -> None:
        # Skip images that are already inside a reference node
        if isinstance(node.parent, nodes.reference):
            return

        # Skip images that have been explicitly excluded
        if 'no-spotlight' in node.get('classes', []):
            return

        # Skip images with no URI
        if not node.get('uri', ''):
            return

        # Create a spotlight_reference node wrapping the image.
        # The actual href will be resolved at write time in visit_spotlight_reference.
        ref = spotlight_reference('', '', internal=False)

        # Replace image node with reference wrapping the image
        node.replace_self([ref])
        ref.append(node)


def add_static_path(app: Sphinx) -> None:
    """Add the extension's _static directory to the Sphinx static path."""
    static_dir = str(_STATIC_DIR)
    static_paths = getattr(app.config, 'html_static_path', None)

    if static_paths is None:
        # Initialize html_static_path if it hasn't been set yet.
        app.config.html_static_path = [static_dir]
        return

    if static_dir not in static_paths:
        static_paths.append(static_dir)


def setup(app: Sphinx) -> dict[str, Any]:
    """Set up the sphinxcontrib.spotlight extension."""
    app.add_node(
        spotlight_reference,
        html=(visit_spotlight_reference, depart_spotlight_reference),
    )
    app.add_post_transform(SpotlightTransform)

    app.connect('builder-inited', add_static_path)
    app.add_js_file('sphinxcontrib_spotlight/js/spotlight.bundle.min.js')

    return {
        'version': '0.1.0',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
