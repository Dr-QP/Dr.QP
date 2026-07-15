"""Fixtures that keep the AI responder workflow security matrix explicit."""

import json
import os
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
VALID_RESPONDERS = {'none', 'claude', 'codex', 'claude,codex'}


def enabled_responders(value: str | None) -> set[str]:
    """Return enabled responders, applying the repository default exactly."""
    configured_value = 'claude' if not value else value
    if configured_value not in VALID_RESPONDERS:
        raise ValueError(f'Unsupported AI_RESPONDERS value: {configured_value}')
    if configured_value == 'none':
        return set()
    return set(configured_value.split(','))


def responder_is_triggered(
    responder: str,
    event_name: str,
    body: str = '',
    title: str = '',
) -> bool:
    """Model the event marker matrix used by both responder workflows."""
    if event_name == 'pull_request':
        return True
    marker = f'@{responder}'
    if event_name == 'issues':
        return marker in body or marker in title
    return marker in body


def agent_execution_allowed(
    responder: str,
    configuration: str | None,
    event_name: str,
    body: str = '',
    title: str = '',
    *,
    authorized: bool = True,
    fork_originated: bool = False,
    protected_path_changed: bool = False,
) -> bool:
    """Model the guard that must be true before either agent gets credentials."""
    return (
        responder in enabled_responders(configuration)
        and responder_is_triggered(responder, event_name, body, title)
        and authorized
        and not fork_originated
        and not protected_path_changed
    )


class FakeGitHub:
    """Record the trusted GitHub calls made by responder workflow fixtures."""

    def __init__(self, pr_head_sha: str) -> None:
        self.pr_head_sha = pr_head_sha
        self.requested_pr_numbers: list[int] = []
        self.replies: list[tuple[int, str]] = []

    def pull_request_head(self, number: int) -> str:
        """Return a mocked pull-request head SHA."""
        self.requested_pr_numbers.append(number)
        return self.pr_head_sha

    def reply(self, number: int, body: str) -> None:
        """Record a threaded responder reply."""
        self.replies.append((number, body))


def checkout_ref(
    event_name: str,
    context_sha: str,
    issue_number: int | None,
    is_pull_request_comment: bool,
    github: FakeGitHub,
) -> str:
    """Resolve PR comment events to their head, leaving issues on context SHA."""
    if event_name == 'issue_comment' and is_pull_request_comment:
        assert issue_number is not None
        return github.pull_request_head(issue_number)
    return context_sha


def sanitized_reply(message: str) -> str:
    """Add the loop marker and keep a model response from retriggering Codex."""
    clean_message = message.replace('@codex', '@\u200bcodex')[:6000]
    return f'<!-- codex-responder -->\n{clean_message}'


@pytest.mark.parametrize(
    ('value', 'expected'),
    [
        (None, {'claude'}),
        ('none', set()),
        ('claude', {'claude'}),
        ('codex', {'codex'}),
        ('claude,codex', {'claude', 'codex'}),
    ],
)
def test_responder_configuration_enables_exactly_the_requested_agents(
    value: str | None, expected: set[str]
) -> None:
    """The default and every supported configuration have one exact meaning."""
    assert enabled_responders(value) == expected


@pytest.mark.parametrize('value', ['Claude', ' codex', 'codex ', 'claude, codex', 'notcodex'])
def test_invalid_responder_configuration_fails_closed(value: str) -> None:
    """Invalid, mixed-case, whitespace, and substring values never enable agents."""
    with pytest.raises(ValueError):
        enabled_responders(value)


@pytest.mark.parametrize(
    ('event_name', 'body', 'title', 'expected'),
    [
        ('issue_comment', '@claude review', '', {'claude'}),
        ('issue_comment', '@codex review', '', {'codex'}),
        ('issue_comment', 'please review', '', set()),
        ('pull_request_review_comment', '@codex inspect this', '', {'codex'}),
        ('pull_request_review', '@claude inspect this', '', {'claude'}),
        ('issues', '', '@codex investigate', {'codex'}),
    ],
)
def test_mentions_start_only_the_matching_responder(
    event_name: str, body: str, title: str, expected: set[str]
) -> None:
    """Marker matching is exact across the common event matrix."""
    actual = {
        responder
        for responder in ('claude', 'codex')
        if responder_is_triggered(responder, event_name, body, title)
    }
    assert actual == expected


@pytest.mark.parametrize(
    ('configuration', 'expected'),
    [
        (None, {'claude'}),
        ('none', set()),
        ('claude', {'claude'}),
        ('codex', {'codex'}),
        ('claude,codex', {'claude', 'codex'}),
    ],
)
def test_automatic_pr_events_run_only_enabled_responders(
    configuration: str | None, expected: set[str]
) -> None:
    """Opening a PR asks exactly the enabled responders for a review."""
    actual = {
        responder
        for responder in ('claude', 'codex')
        if agent_execution_allowed(responder, configuration, 'pull_request')
    }
    assert actual == expected


@pytest.mark.parametrize('responder', ['claude', 'codex'])
def test_untrusted_events_never_reach_an_agent_with_credentials(responder: str) -> None:
    """Protected paths, forks, and unauthorized users fail closed for both agents."""
    for guard in (
        {'authorized': False},
        {'fork_originated': True},
        {'protected_path_changed': True},
    ):
        assert not agent_execution_allowed(
            responder,
            'claude,codex',
            'issue_comment',
            f'@{responder} review',
            **guard,
        )


def test_pr_comment_checkout_uses_mocked_pr_head_and_issues_keep_context_ref() -> None:
    """A comment on a PR never checks out the default-branch context SHA."""
    github = FakeGitHub(pr_head_sha='pr-head')

    assert checkout_ref('issue_comment', 'main-sha', 42, True, github) == 'pr-head'
    assert checkout_ref('issues', 'main-sha', 42, False, github) == 'main-sha'
    assert github.requested_pr_numbers == [42]


def test_non_review_reply_is_sanitized_and_cannot_retrigger_itself() -> None:
    """Mock the reply client and ensure the responder marker cannot loop."""
    github = FakeGitHub(pr_head_sha='unused')
    reply = sanitized_reply('@codex please run this again')
    github.reply(42, reply)

    assert github.replies == [(42, reply)]
    assert '<!-- codex-responder -->' in reply
    assert '@codex' not in reply
    assert not responder_is_triggered('codex', 'issue_comment', reply)


@pytest.mark.parametrize('workflow_name', ['claude-review.yml', 'codex-review.yml'])
def test_workflows_validate_configuration_and_protect_agent_credentials(
    workflow_name: str,
) -> None:
    """Both workflows retain the shared configuration and credential guardrails."""
    workflow = (ROOT / '.github' / 'workflows' / workflow_name).read_text()

    assert "AI_RESPONDERS: ${{ vars.AI_RESPONDERS || 'claude' }}" in workflow
    assert 'Validate AI_RESPONDERS' in workflow
    assert 'Unsupported AI_RESPONDERS value' in workflow
    assert "contains(format(',{0},', vars.AI_RESPONDERS || 'claude')" in workflow
    assert 'github.event.pull_request.head.repo.full_name == github.repository' in workflow
    assert "steps.workflow_changed.outputs.workflowChanged == 'false'" in workflow
    assert '.github/workflows/codex-review.yml' in workflow
    assert '.github/workflows/claude-review.yml' in workflow
    assert '.github/actions/' in workflow
    assert 'contents: write' not in workflow
    assert 'id-token: write' not in workflow


def test_codex_action_runs_in_the_ros_container_with_step_scoped_credentials() -> None:
    """Codex uses the container and receives secrets only through its own step."""
    workflow = (ROOT / '.github' / 'workflows' / 'codex-review.yml').read_text()

    assert 'image: ghcr.io/dr-qp/jazzy-ros-desktop:edge' in workflow
    assert 'openai/codex-action@52fe01ec70a42f454c9d2ebd47598f9fd6893d56' in workflow
    assert 'openai-api-key: ${{ secrets.OPENAI_API_KEY }}' in workflow
    assert 'sandbox: danger-full-access' in workflow
    assert 'safety-strategy: drop-sudo' in workflow
    assert 'allow-bots: false' in workflow
    assert 'scripts/with-ros-env.sh' in workflow
    assert 'OPENAI_API_KEY:' not in workflow.split('jobs:', maxsplit=1)[0]
    assert 'CODEX_API_KEY:' not in workflow.split('jobs:', maxsplit=1)[0]


def test_post_review_script_uses_one_mocked_review_request(tmp_path: Path) -> None:
    """The fallback publisher creates one review with all validated comments."""
    script = (ROOT / 'scripts' / 'post-review.sh').read_text()

    assert 'pulls/${pr_number}/reviews' in script
    assert 'comments' in script
    assert 'gh api' in script

    summary_file = tmp_path / 'summary.md'
    comments_file = tmp_path / 'comments.json'
    call_file = tmp_path / 'gh-call.json'
    mock_gh = tmp_path / 'gh'
    summary_file.write_text('<!-- codex-responder -->\nNo issues found.\n')
    comments_file.write_text('[{"path":"example.py","line":4,"side":"RIGHT","body":"A finding."}]')
    mock_gh.write_text(
        '#!/usr/bin/env bash\n'
        'set -euo pipefail\n'
        'printf \'%s\\n\' "$@" > "$GH_CALL_FILE.args"\n'
        'cat > "$GH_CALL_FILE.payload"\n'
    )
    mock_gh.chmod(0o755)
    environment = os.environ | {
        'PATH': f'{tmp_path}:{os.environ["PATH"]}',
        'GH_CALL_FILE': str(call_file),
    }

    subprocess.run(
        [
            str(ROOT / 'scripts' / 'post-review.sh'),
            '--repo',
            'Dr-QP/Dr.QP',
            '--pr',
            '42',
            '--event',
            'COMMENT',
            '--summary-file',
            str(summary_file),
            '--comments-file',
            str(comments_file),
        ],
        check=True,
        env=environment,
    )

    assert Path(f'{call_file}.args').read_text().splitlines() == [
        'api',
        '--method',
        'POST',
        'repos/Dr-QP/Dr.QP/pulls/42/reviews',
        '--input',
        '-',
    ]
    assert json.loads(Path(f'{call_file}.payload').read_text()) == {
        'body': '<!-- codex-responder -->\nNo issues found.\n',
        'comments': [
            {
                'body': 'A finding.',
                'line': 4,
                'path': 'example.py',
                'side': 'RIGHT',
            }
        ],
        'event': 'COMMENT',
    }
