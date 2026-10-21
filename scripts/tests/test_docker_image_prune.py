from pathlib import Path
import shutil
import subprocess

REPO_ROOT = Path(__file__).parents[2]
SCRIPT = REPO_ROOT / '.devcontainer/scripts/docker-image-prune.sh'
PRUNE_ARGS = 'image prune -af --filter until=72h'
RECLAIMED = 'Total reclaimed space: 1.5GB'


def run_prune(
    tmp_path: Path, *, docker_installed: bool = True, info_exit: int = 0, prune_exit: int = 0
) -> tuple[subprocess.CompletedProcess[str], list[str]]:
    """Run the script with a stub docker that logs its arguments."""
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    for tool in ('bash', 'env'):
        (fake_bin / tool).symlink_to(shutil.which(tool))
    log = tmp_path / 'docker.log'
    if docker_installed:
        (fake_bin / 'docker').write_text(
            '#!/usr/bin/env bash\n'
            f'echo "$*" >> "{log}"\n'
            f'if [[ "$1" == info ]]; then exit {info_exit}; fi\n'
            f'if [[ "$*" == "{PRUNE_ARGS}" ]]; then\n'
            '  echo "Deleted Images:"\n'
            '  echo "deleted: sha256:0123"\n'
            f'  echo "{RECLAIMED}"\n'
            f'  exit {prune_exit}\n'
            'fi\n'
        )
        (fake_bin / 'docker').chmod(0o755)
    result = subprocess.run([SCRIPT], env={'PATH': str(fake_bin)}, text=True, capture_output=True)
    calls = log.read_text().splitlines() if log.exists() else []
    return result, calls


def test_prunes_unused_images_older_than_72h(tmp_path):
    result, calls = run_prune(tmp_path)
    assert result.returncode == 0, result.stderr
    assert calls[-1] == PRUNE_ARGS
    assert result.stdout.strip() == RECLAIMED


def test_skips_when_docker_is_missing(tmp_path):
    result, calls = run_prune(tmp_path, docker_installed=False)
    assert result.returncode == 0, result.stderr
    assert calls == []


def test_skips_when_daemon_is_unreachable(tmp_path):
    result, calls = run_prune(tmp_path, info_exit=1)
    assert result.returncode == 0, result.stderr
    assert PRUNE_ARGS not in calls


def test_warns_and_succeeds_when_prune_fails(tmp_path):
    result, calls = run_prune(tmp_path, prune_exit=1)
    assert result.returncode == 0, result.stderr
    assert calls[-1] == PRUNE_ARGS
    assert 'Warning' in result.stderr
