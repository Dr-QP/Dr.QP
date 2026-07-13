#!/usr/bin/env python3

"""Regression tests for the ShellCheck autofix script."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess


def _run_shellcheck_fix(repo_path: Path, fake_bin: Path) -> subprocess.CompletedProcess[str]:
    """Run the autofix script in an isolated repository with a fake ShellCheck."""
    repo_root = Path(__file__).resolve().parents[3]
    environment = os.environ.copy()
    environment['PATH'] = f'{fake_bin}:{environment["PATH"]}'
    return subprocess.run(
        [str(repo_root / 'scripts' / 'shellcheck-fix.sh')],
        cwd=repo_path,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )


def test_shellcheck_fix_succeeds_after_applying_findings_diff(
    repo_tmp_path: Path,
) -> None:
    """ShellCheck's findings status should not fail an applied autofix."""
    sample_script = repo_tmp_path / 'sample.sh'
    sample_script.write_text('#!/usr/bin/env bash\nvalue=$name\n')

    fake_bin = repo_tmp_path / 'fake-bin'
    fake_bin.mkdir()
    fake_shellcheck = fake_bin / 'shellcheck'
    fake_shellcheck.write_text(
        """#!/usr/bin/env bash
cat <<'PATCH'
diff --git a/sample.sh b/sample.sh
--- a/sample.sh
+++ b/sample.sh
@@ -1,2 +1,2 @@
 #!/usr/bin/env bash
-value=$name
+value="$name"
PATCH
exit 1
"""
    )
    fake_shellcheck.chmod(0o755)
    subprocess.run(['git', 'init', '--quiet'], cwd=repo_tmp_path, check=True)

    completed = _run_shellcheck_fix(repo_tmp_path, fake_bin)

    assert sample_script.read_text() == '#!/usr/bin/env bash\nvalue="$name"\n'
    assert completed.returncode == 0, completed.stderr
    assert list((repo_tmp_path / '.tmp').iterdir()) == []


def test_shellcheck_fix_propagates_execution_failure(repo_tmp_path: Path) -> None:
    """ShellCheck execution errors should fail without applying their output."""
    sample_script = repo_tmp_path / 'sample.sh'
    original_content = '#!/usr/bin/env bash\nvalue=$name\n'
    sample_script.write_text(original_content)

    fake_bin = repo_tmp_path / 'fake-bin'
    fake_bin.mkdir()
    fake_shellcheck = fake_bin / 'shellcheck'
    fake_shellcheck.write_text('#!/usr/bin/env bash\nprintf "invalid diff\\n"\nexit 2\n')
    fake_shellcheck.chmod(0o755)

    completed = _run_shellcheck_fix(repo_tmp_path, fake_bin)

    assert completed.returncode == 2
    assert sample_script.read_text() == original_content
    assert list((repo_tmp_path / '.tmp').iterdir()) == []
