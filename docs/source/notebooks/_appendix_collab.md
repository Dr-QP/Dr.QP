---
jupytext:
  text_representation:
    extension: .md
    format_name: myst
    format_version: 0.13
    jupytext_version: 1.17.1
kernelspec:
  display_name: .venv
  language: python
  name: python3
---

### Convenient links to editors

The code below is provided for your convenience to open this notebook in one of the editors.

```{code-cell} ipython3
:tags: [keep_output]

from IPython.display import display, Markdown

source_branch = 'main'  ## <<<< source branch name

codespaces_url = f'https://github.com/codespaces/new?machine=basicLinux32gb&repo=96508363&workspace=%2Fhome%2Frosdev%2Fros2_ws%2FDr.QP.code-workspace&ref={source_branch}&geo=UsWest&devcontainer_path=.devcontainer%2Fprebuilt%2Fdevcontainer.json'

notebook_path = (
    f'Dr-QP/Dr.QP/blob/{source_branch}/docs/source/notebooks/1_getting_started_with_robot_ik.ipynb'
)
colab_badge_markdown = f'[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/{notebook_path})'
github_badge_markdown = f'[![Open In GitHub](https://img.shields.io/badge/Open%20In-GitHub-blue.svg)](https://github.com/{notebook_path})'
codespace_badge_markdown = f'[![Open In GitHub Codespace](https://img.shields.io/badge/Open%20In-GitHub%20Codespace-blue.svg)]({codespaces_url})'
badge_markdown = f'{colab_badge_markdown}\n\n{github_badge_markdown}\n\n{codespace_badge_markdown}'

display(Markdown(badge_markdown))
```

### Viewing on github

All cell outputs in this notebook are stripped from source code, so github will not show them. To see the outputs, run the notebook locally, on Colab or in GitHub Codespace.

### Colab specific setup

Google Colab opens only the notebook file and all the dependencies are not available. The code below will clone the repository and install the dependencies.

In order to view non default branch change `source_branch='main'` above and rerun the cell.

#### Runtime restart

The runtime need to be restarted to pick up the new modules. The code below will install them and kill runtime, simply run all cells again afterwards

```{code-cell} ipython3
# type: ignore
# Setup for Google Colab
import importlib.util
import os

IN_COLAB = (
    importlib.util.find_spec('google') is not None
    and importlib.util.find_spec('google.colab') is not None
)

if IN_COLAB:
    try:
        import plotting  # noqa: F401
        import point  # noqa: F401
    except ImportError:
        !git clone --filter=blob:none --no-checkout --depth 1 --sparse https://github.com/Dr-QP/Dr.QP.git --branch=$source_branch
        !cd Dr.QP && git sparse-checkout add notebooks && git checkout && cd ..
        !mv -f Dr.QP/* .
        !mv -f docs/source/notebooks/* .
        !rm -rf Dr.QP
        %pip install -r requirements.txt

        print('\n\n\nRestarting runtime to pick up the new modules...')
        os.kill(os.getpid(), 9)
```
