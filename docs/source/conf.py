"""
Configuration file for the Sphinx documentation builder.

This file only contains a selection of the most common options. For a full
list see the documentation:
https://www.sphinx-doc.org/en/master/usage/configuration.html
"""

import os
from typing import Any, Dict
from sphinx.application import Sphinx

# -- Project information

project = 'Dr.QP'
copyright = '2017-2025 Anton Matosov'  # noqa
author = 'Anton Matosov'

# Remove `release` to avoid their display in the title
# release = '0.1'
version = '0.1.0'


# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
#
# The master toctree document.
master_doc = 'index'

# The default role
default_role = 'any'

# The set of warnings to suppress.
suppress_warnings = ['image.nonlocal_uri']

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = 'en'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This patterns also effect to html_static_path and html_extra_path
exclude_patterns = [
    '**/_*.rst',
    '**/_*.md',
    '**/*.ipynb',
    '_build',
    'Thumbs.db',
    '.DS_Store',
    '**.ipynb_checkpoints',
]

# -- General configuration
extensions = [
    'sphinx_copybutton',
    'sphinx_design',
    'sphinx_togglebutton',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.doctest',
    'sphinx.ext.duration',
    'sphinx.ext.intersphinx',
    'sphinxext.rediraffe',
    'sphinxcontrib.lightbox2',
    'myst_nb',  # for embedding jupyter notebooks
    # Disabled for now due to conflict with myst_nb
    # see https://github.com/executablebooks/MyST-NB/issues/421
    # and https://github.com/executablebooks/MyST-NB/issues/304
    # myst_parser is used by myst_nb under the hood
    # https://github.com/executablebooks/MyST-NB/blob/master/myst_nb/sphinx_ext.py#L49
    # so it is not really needed here
    # 'myst_parser',  # for embedding myst pages
]

# Javascript to be loaded on pages containing ipywidgets
nb_ipywidgets_js = {
    'https://cdnjs.cloudflare.com/ajax/libs/require.js/2.3.4/require.min.js': {
        'integrity': 'sha256-Ae2Vz/4ePdIu6ZyI/5ZGsYnb+m0JlOmKPjt6XZ9JJkA=',
        'crossorigin': 'anonymous',
    },
    'https://cdn.jsdelivr.net/npm/@jupyter-widgets/html-manager@1.0.6/dist/embed-amd.js': {
        'data-jupyter-widgets-cdn': 'https://cdn.jsdelivr.net/npm/',
        'crossorigin': 'anonymous',
    },
}
nb_execution_timeout = 300  # seconds

myst_enable_extensions = [
    'amsmath',
    'colon_fence',
    'deflist',
    'dollarmath',
    'html_image',
]
myst_url_schemes = ('http', 'https', 'mailto')


intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

templates_path = ['_templates']

lightbox2_wrap_around = False
lightbox2_fit_images_in_viewport = True

# -- Redirects -----------------------------------------------------------------

rediraffe_branch = 'origin/main'
rediraffe_redirects = 'redirects.txt'

# -- Sitemap -----------------------------------------------------------------

# ReadTheDocs has its own way of generating sitemaps, etc.
if not os.environ.get('READTHEDOCS'):
    extensions += ['sphinx_sitemap']

    html_baseurl = os.environ.get('SITEMAP_URL_BASE', 'http://127.0.0.1:8000/')
    sitemap_locales = [None]
    sitemap_url_scheme = '{link}'

# -- Options for HTML output ---------------------------------------------------

html_theme = 'pydata_sphinx_theme'
html_theme_options = {
    'github_url': 'https://github.com/Dr-QP/Dr.QP',
    'secondary_sidebar_items': {
        'index': [],
        '**/*': ['page-toc', 'edit-this-page', 'sourcelink'],
    },
    'navbar_center': ['navbar-nav'],  # 'version-switcher',
    'footer_start': ['copyright'],
    'footer_center': ['sphinx-version'],
}
html_static_path = ['_static']
html_css_files = [
    'styles/custom.css',
]

# -- Edit on GitHub -------------------------------------------------------------

github_user = 'dr-qp'
github_repo = 'Dr.QP'


def get_github_branch_from_pr(pr_number: int):
    from github import Github

    g = Github()
    return g.get_repo(github_user + '/' + github_repo).get_pull(pr_number).head.ref


def version_name():
    rtd_version_name = os.environ.get('READTHEDOCS_VERSION_NAME', 'main')
    rtd_version_type = os.environ.get('READTHEDOCS_VERSION_TYPE', '')
    rtd_git_sha = os.environ.get('READTHEDOCS_GIT_COMMIT_HASH', 'main')

    if rtd_version_type == 'tag' or rtd_version_type == 'branch':
        return rtd_version_name
    elif rtd_version_type == 'external':  # pull request
        pr_number = int(rtd_version_name)
        return get_github_branch_from_pr(pr_number) or rtd_git_sha

    return 'main'


html_context = {
    'display_github': True,
    'github_user': github_user,
    'github_repo': github_repo,
    'github_version': version_name(),
    'conf_py_path': '/docs/source/',
    'doc_path': '/docs/source/',
}

# -- Options for EPUB output ---------------------------------------------------
epub_show_urls = 'footnote'


# -- application setup -------------------------------------------------------


def setup(app: Sphinx) -> Dict[str, Any]:
    """
    Add custom configuration to sphinx app.

    Args:
    -----
        app: the Sphinx application

    """
    # Set default if not already defined in the shell
    os.environ.setdefault('SPHINX_BUILD', '1')

    return {
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
