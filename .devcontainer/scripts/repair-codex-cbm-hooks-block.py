#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# ///
"""
Move foreign tables out of codebase-memory-mcp's SessionStart block in Codex's config.toml.

Why and when: see bugs/cbm-codex-hooks-block-foreign-tables.
"""

import os
from pathlib import Path
import re
import tempfile

OPEN_MARKER = '# >>> codebase-memory-mcp SessionStart >>>'
CLOSE_MARKER = '# <<< codebase-memory-mcp SessionStart <<<'
OWNED_HEADER = re.compile(r'\[\[hooks\.(SessionStart|SubagentStart)(\.hooks)?\]\]')
HEADER = re.compile(r'\[')


def codex_home() -> Path:
    """Return the Codex home directory, honoring ``CODEX_HOME``."""
    return Path(os.environ.get('CODEX_HOME') or Path.home() / '.codex')


def foreign_start(body: list[str]) -> int:
    """Return the index in ``body`` where the first foreign table and its leading comments begin."""
    for index, line in enumerate(body):
        stripped = line.strip()
        if HEADER.match(stripped) and not OWNED_HEADER.fullmatch(stripped):
            while index > 0 and body[index - 1].lstrip().startswith('#'):
                index -= 1
            return index
    return len(body)


def strip_blank_edges(lines: list[str]) -> list[str]:
    """Return ``lines`` without leading or trailing blank lines."""
    while lines and not lines[0].strip():
        lines = lines[1:]
    while lines and not lines[-1].strip():
        lines = lines[:-1]
    return lines


def repair(current: str) -> str:
    """Return ``current`` with the hooks block holding only codebase-memory-mcp's own tables."""
    lines = current.splitlines(keepends=True)
    stripped = [line.rstrip('\n') for line in lines]
    if OPEN_MARKER not in stripped:
        return current
    open_index = stripped.index(OPEN_MARKER)
    if CLOSE_MARKER not in stripped[open_index:]:
        return current
    close_index = stripped.index(CLOSE_MARKER, open_index)

    body = lines[open_index + 1 : close_index]
    split = foreign_start(body)
    owned = strip_blank_edges(body[:split])
    foreign = strip_blank_edges(body[split:])
    after = lines[close_index + 1 :]

    close = lines[close_index]
    moved = ['\n', *foreign] if foreign else []
    if moved:
        close = close.rstrip('\n') + '\n'
        if after and after[0].strip():
            moved.append('\n')
    return ''.join(lines[: open_index + 1] + owned + [close] + moved + after)


def write_atomically(path: Path, content: str) -> None:
    """Replace ``path`` with ``content`` through a temporary file beside it, keeping its mode."""
    fd, temp_name = tempfile.mkstemp(dir=path.parent, prefix=f'.{path.name}.')
    temp_path = Path(temp_name)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as temp_file:
            temp_file.write(content)
        temp_path.chmod(path.stat().st_mode & 0o777)
        temp_path.replace(path)
    except BaseException:
        temp_path.unlink(missing_ok=True)
        raise


def main() -> None:
    """Repair the hooks block in the current Codex home's config, if it needs it."""
    config = codex_home() / 'config.toml'
    if not config.is_file():
        return
    current = config.read_text(encoding='utf-8')
    repaired = repair(current)
    if repaired != current:
        write_atomically(config, repaired)
        print(f'moved foreign tables out of the codebase-memory-mcp hooks block in {config}')


if __name__ == '__main__':
    main()
