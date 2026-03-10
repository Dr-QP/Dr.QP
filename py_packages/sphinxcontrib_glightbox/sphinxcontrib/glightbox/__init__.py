"""
Sphinx extension for GLightbox - a pure JavaScript lightbox library.

This extension wraps images in the documentation with GLightbox links,
providing a better lightbox experience than the older lightbox2 library.
It properly handles SVG images without rendering them oversized.
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

_INIT_SCRIPT = """
document.addEventListener('DOMContentLoaded', function() {
    if (typeof GLightbox !== 'undefined') {
        const lightbox = GLightbox({
            selector: '.glightbox',
            touchNavigation: true,
            loop: false,
            zoomable: true,
        });
    }
});
"""


class glightbox_reference(nodes.reference):
    """
    Custom reference node for GLightbox-wrapped images.

    Using a custom node allows us to defer URI resolution to write time,
    so that the href correctly points to the output path in _images/ rather
    than the original source path.
    """


def visit_glightbox_reference(self: HTML5Translator, node: glightbox_reference) -> None:
    """Render a glightbox reference as an <a> tag with the correct image URI."""
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

    atts: dict[str, Any] = {'class': 'glightbox', 'href': href}

    # Pass through data-* attributes for GLightbox options (e.g., data-title)
    for key, val in node.attval.items() if hasattr(node, 'attval') else []:
        if key.startswith('data-'):
            atts[key] = val

    # Add alt text as glightbox description if available
    for child in node.children:
        if isinstance(child, nodes.image):
            alt = child.get('alt', '')
            if alt:
                atts['data-glightbox'] = f'description: {alt}'
            break

    self.body.append(self.starttag(node, 'a', '', **atts))


def depart_glightbox_reference(self: HTML5Translator, node: glightbox_reference) -> None:
    """Close the <a> tag for the glightbox reference."""
    self.body.append('</a>')


class GlightboxTransform(SphinxPostTransform):
    """Post-transform that wraps image nodes with GLightbox anchor tags."""

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
        if 'no-glightbox' in node.get('classes', []):
            return

        # Skip images with no URI
        if not node.get('uri', ''):
            return

        # Create a glightbox_reference node wrapping the image.
        # The actual href will be resolved at write time in visit_glightbox_reference.
        ref = glightbox_reference('', '', internal=False)

        # Replace image node with reference wrapping the image
        node.replace_self([ref])
        ref.append(node)


def add_static_path(app: Sphinx) -> None:
    """Add the extension's _static directory to the Sphinx static path."""
    app.config.html_static_path.append(str(_STATIC_DIR))


def setup(app: Sphinx) -> dict[str, Any]:
    """Set up the sphinxcontrib.glightbox extension."""
    app.add_node(
        glightbox_reference,
        html=(visit_glightbox_reference, depart_glightbox_reference),
    )
    app.add_post_transform(GlightboxTransform)

    app.connect('builder-inited', add_static_path)
    app.add_css_file('sphinxcontrib_glightbox/css/glightbox.min.css')
    app.add_js_file('sphinxcontrib_glightbox/js/glightbox.min.js')
    app.add_js_file(None, body=_INIT_SCRIPT, type='text/javascript')

    return {
        'version': '0.1.0',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
