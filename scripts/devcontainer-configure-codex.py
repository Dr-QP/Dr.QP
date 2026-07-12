#!/usr/bin/env python3
"""Configure Codex's persistent user settings inside the devcontainer."""

import os
from pathlib import Path
import tempfile

import toml


def main() -> None:
    """Write devcontainer-only Codex settings to the user's config file."""
    codex_home = Path(os.environ.get('CODEX_HOME', Path.home() / '.codex'))
    config_path = codex_home / 'config.toml'

    codex_home.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(codex_home, 0o700)
    config = toml.load(config_path) if config_path.exists() else {}
    config['sandbox_mode'] = 'danger-full-access'
    config['approval_policy'] = 'never'

    with tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        dir=codex_home,
        prefix='config.toml.',
        delete=False,
    ) as config_file:
        toml.dump(config, config_file)
        temporary_path = Path(config_file.name)

    temporary_path.chmod(0o600)
    temporary_path.replace(config_path)


if __name__ == '__main__':
    main()
