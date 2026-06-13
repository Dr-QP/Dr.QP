#!/usr/bin/env python3
#
# Copyright (c) 2017-2025 Anton Matosov
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import fcntl
import os
from pathlib import Path


class InstanceAlreadyRunningError(RuntimeError):
    """Raised when another instance already holds an instance guard lock."""


class InstanceGuard:
    """Hold an advisory file lock to prevent duplicate local process instances."""

    def __init__(self, name: str, lock_dir: Path | str = Path('.tmp')):
        self.name = name
        self._lock_path = Path(lock_dir) / f'{name}.lock'
        self._lock_file = None

    @property
    def lock_path(self) -> Path:
        """Return the lock file path used by this guard."""
        return self._lock_path

    def acquire(self) -> None:
        """Acquire the instance lock or raise when another process owns it."""
        self._lock_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock_file = self._lock_path.open('w', encoding='utf-8')
        try:
            fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self._lock_file.close()
            self._lock_file = None
            raise InstanceAlreadyRunningError(
                f'Another {self.name} process already holds the startup lock.'
            ) from exc

        self._lock_file.seek(0)
        self._lock_file.truncate()
        self._lock_file.write(f'{os.getpid()}\n')
        self._lock_file.flush()

    def release(self) -> None:
        """Release the lock if it is currently held."""
        if self._lock_file is None:
            return
        try:
            fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_UN)
        finally:
            self._lock_file.close()
            self._lock_file = None

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        self.release()
