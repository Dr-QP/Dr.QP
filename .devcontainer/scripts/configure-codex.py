#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = ["tomlkit"]
# ///
"""
Set Codex's full-access sandbox and approval policy in its config.toml.

Decision and constraints: see architecture/codex-full-access-in-devcontainer.
"""

import os
from pathlib import Path
import tempfile

import tomlkit

MANAGED_SETTINGS: dict[str, str] = {
    'sandbox_mode': 'danger-full-access',
    'approval_policy': 'never',
}


def codex_home() -> Path:
    """Return the Codex home directory, honoring ``CODEX_HOME``."""
    return Path(os.environ.get('CODEX_HOME') or Path.home() / '.codex')


def render_config(current: str) -> str:
    """Return ``current`` with the managed top-level keys set, comments preserved."""
    document = tomlkit.parse(current)
    missing = tomlkit.document()
    for key, value in MANAGED_SETTINGS.items():
        if key in document:
            document[key] = value
        else:
            missing[key] = value
    rest = tomlkit.dumps(document)
    if not missing:
        return rest
    # New keys go first: tomlkit would append them after a leading comment block,
    # inside another tool's marker-delimited section.
    return tomlkit.dumps(missing) + ('\n' + rest if rest else '')


def write_atomically(path: Path, content: str) -> None:
    """Replace ``path`` with ``content`` through a ``0600`` temporary file beside it."""
    fd, temp_name = tempfile.mkstemp(dir=path.parent, prefix=f'.{path.name}.')
    temp_path = Path(temp_name)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as temp_file:
            temp_file.write(content)
        temp_path.chmod(0o600)
        temp_path.replace(path)
    except BaseException:
        temp_path.unlink(missing_ok=True)
        raise


def main() -> None:
    """Configure Codex in the current Codex home."""
    home = codex_home()
    home.mkdir(parents=True, exist_ok=True)
    home.chmod(0o700)
    config = home / 'config.toml'
    current = config.read_text(encoding='utf-8') if config.exists() else ''
    write_atomically(config, render_config(current))


if __name__ == '__main__':
    main()
