"""
Shape gate for plan checkboxes in `docs/knowledge/data/plans/`.

Two rules, both about shape only — this gate can never tell whether an evidence
claim is *true*, which is the verify skill's job and a human's:

- **Evidence**: in an active plan (no `stage` in frontmatter), every ticked
  `- [x]` task under `## Implementation Steps` carries an indented
  `- **Evidence:**` child with non-empty content.
- **Completeness**: a plan with `stage: done` has no unchecked `- [ ]` task.

Closed plans are exempt from the evidence rule. They are historical records, and
backfilling evidence into them would be inventing it — the exact failure this
gate exists to prevent.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re

import frontmatter
import pytest

PLANS_DIR = Path(__file__).resolve().parents[1] / 'data' / 'plans'

# The plan skill specifies `## Implementation Steps`. `## Tasks` is the older
# heading still used by plans written before it, and a gate that silently skips
# a whole document is the invisibility this gate exists to remove.
TASK_SECTIONS = frozenset({'Implementation Steps', 'Tasks'})

TASK_LINE = re.compile(r'^- \[(?P<mark>[ xX])\]\s+(?P<text>.*)$')
EVIDENCE_LINE = re.compile(r'^(?P<indent>[ \t]+)- \*\*Evidence:\*\*(?P<body>.*)$')
LIST_ITEM = re.compile(r'^(?P<indent>[ \t]+)(?:[-+*]|\d+[.)])\s+')
FENCE = re.compile(r'^[ \t]*(?:```|~~~)')
HEADING = re.compile(r'^(?P<hashes>#{1,6})\s+(?P<title>.*)$')

RULE_EVIDENCE = 'evidence'
RULE_COMPLETENESS = 'completeness'


@dataclass(frozen=True)
class Violation:
    """A single rule breach, anchored to the line that must change."""

    path: Path
    line: int
    rule: str
    message: str

    def __str__(self) -> str:
        """Render the violation as `path:line [rule] message`."""
        return f'{self.path}:{self.line} [{self.rule}] {self.message}'


@dataclass
class TaskBlock:
    """A checkbox task line plus the indented lines that belong to it."""

    line: int
    ticked: bool
    text: str
    body: list[str]


def task_blocks(text: str) -> list[TaskBlock]:
    """
    Collect the checkbox tasks under a plan's task section.

    Checkboxes elsewhere in the document — the format example under
    `## Approach`, say — are not tasks, and neither is anything inside a fenced
    code block.

    Args:
        text: The full Markdown source of a plan document.

    Returns:
        The task blocks in document order.

    """
    blocks: list[TaskBlock] = []
    current: TaskBlock | None = None
    section: str | None = None
    in_fence = False

    for lineno, line in enumerate(text.splitlines(), start=1):
        if FENCE.match(line):
            in_fence = not in_fence
            if current is not None:
                current.body.append(line)
            continue
        if in_fence:
            if current is not None:
                current.body.append(line)
            continue

        heading = HEADING.match(line)
        if heading:
            current = None
            if len(heading.group('hashes')) <= 2:
                section = heading.group('title').strip()
            continue

        if section not in TASK_SECTIONS:
            current = None
            continue

        task = TASK_LINE.match(line)
        if task:
            current = TaskBlock(
                line=lineno,
                ticked=task.group('mark').lower() == 'x',
                text=task.group('text').strip(),
                body=[],
            )
            blocks.append(current)
            continue

        if current is None:
            continue
        if not line.strip() or line.startswith((' ', '\t')):
            current.body.append(line)
        else:
            current = None

    return blocks


def evidence_text(block: TaskBlock) -> str | None:
    """
    Extract a task's evidence claim.

    Args:
        block: The task block to inspect.

    Returns:
        The claim with its wrapped continuation lines joined, an empty string
        when the marker is present but carries no content, or `None` when there
        is no `- **Evidence:**` line at all.

    """
    for index, line in enumerate(block.body):
        marker = EVIDENCE_LINE.match(line)
        if marker is None:
            continue
        marker_indent = len(marker.group('indent').expandtabs())
        parts = [marker.group('body').strip()]
        for continuation in block.body[index + 1 :]:
            if not continuation.strip() or not continuation.startswith((' ', '\t')):
                break
            list_item = LIST_ITEM.match(continuation)
            if list_item and len(list_item.group('indent').expandtabs()) <= marker_indent:
                break
            if EVIDENCE_LINE.match(continuation):
                break
            parts.append(continuation.strip())
        return ' '.join(part for part in parts if part).strip()
    return None


def check_plan(path: Path, text: str) -> list[Violation]:
    """
    Apply both checkbox rules to one plan document.

    Args:
        path: The path reported in violations.
        text: The full Markdown source of the plan.

    Returns:
        Every violation found, in document order — not just the first.

    """
    stage = frontmatter.loads(text).get('stage')
    violations: list[Violation] = []

    for block in task_blocks(text):
        if stage is None and block.ticked:
            claim = evidence_text(block)
            if claim is None:
                violations.append(
                    Violation(
                        path,
                        block.line,
                        RULE_EVIDENCE,
                        'ticked task has no indented `- **Evidence:**` line',
                    )
                )
            elif not claim:
                violations.append(
                    Violation(
                        path,
                        block.line,
                        RULE_EVIDENCE,
                        'ticked task has an empty `- **Evidence:**` line',
                    )
                )
        elif stage == 'done' and not block.ticked:
            violations.append(
                Violation(
                    path,
                    block.line,
                    RULE_COMPLETENESS,
                    'plan is `stage: done` but this task is unchecked',
                )
            )

    return violations


def check_directory(plans_dir: Path) -> list[Violation]:
    """
    Apply both checkbox rules to every plan in a directory.

    Args:
        plans_dir: Directory holding `*.md` plan documents.

    Returns:
        Every violation found across all plans, grouped by file.

    """
    violations: list[Violation] = []
    for path in sorted(plans_dir.glob('*.md')):
        violations.extend(check_plan(path, path.read_text(encoding='utf-8')))
    return violations


def write_plan(plans_dir: Path, name: str, frontmatter_lines: str, body: str) -> Path:
    """
    Write a fixture plan and return its path.

    Args:
        plans_dir: Directory to write into.
        name: File stem, without the `.md` suffix.
        frontmatter_lines: YAML body of the frontmatter block.
        body: Markdown after the frontmatter.

    Returns:
        The path written.

    """
    plans_dir.mkdir(parents=True, exist_ok=True)
    path = plans_dir / f'{name}.md'
    path.write_text(f'---\n{frontmatter_lines}\n---\n\n{body}', encoding='utf-8')
    return path


def line_of(path: Path, needle: str) -> int:
    """
    Find the 1-based line number of the only line containing `needle`.

    Args:
        path: File to search.
        needle: Substring that must appear on exactly one line.

    Returns:
        The 1-based line number.

    """
    matches = [
        number
        for number, line in enumerate(path.read_text(encoding='utf-8').splitlines(), start=1)
        if needle in line
    ]
    assert len(matches) == 1, f'{needle!r} matched {len(matches)} lines in {path}'
    return matches[0]


ACTIVE_FRONTMATTER = 'type: plan\ncreated: 2026-01-01'
DONE_FRONTMATTER = 'type: plan\ncreated: 2026-01-01\nstage: done\ncompleted: 2026-01-02'


@pytest.fixture
def plans_dir(tmp_path: Path) -> Path:
    """Provide an empty plans directory independent of this repository."""
    return tmp_path / 'plans'


def test_ticked_task_without_evidence_is_a_violation(plans_dir: Path) -> None:
    """A tick with nothing behind it is what this gate exists to catch."""
    path = write_plan(
        plans_dir,
        '20260101-no-evidence',
        ACTIVE_FRONTMATTER,
        '# No evidence\n\n## Implementation Steps\n\n- [x] **1. Push the branch.**\n',
    )

    violations = check_plan(path, path.read_text(encoding='utf-8'))

    assert [(v.line, v.rule) for v in violations] == [
        (line_of(path, '1. Push the branch.'), RULE_EVIDENCE)
    ]


def test_ticked_task_with_evidence_passes(plans_dir: Path) -> None:
    """A tick whose child names a test run satisfies the evidence rule."""
    path = write_plan(
        plans_dir,
        '20260101-with-evidence',
        ACTIVE_FRONTMATTER,
        '# With evidence\n'
        '\n'
        '## Implementation Steps\n'
        '\n'
        '- [x] **1. Push the branch.** Wrapped task prose continues\n'
        '  onto a second line before the child bullet.\n'
        '  - **Evidence:** `git push` succeeded; CI green at `abc1234`.\n',
    )

    assert check_plan(path, path.read_text(encoding='utf-8')) == []


def test_empty_evidence_marker_is_a_violation(plans_dir: Path) -> None:
    """The marker alone is not a claim, so shape compliance does not pass."""
    path = write_plan(
        plans_dir,
        '20260101-empty-evidence',
        ACTIVE_FRONTMATTER,
        '# Empty evidence\n'
        '\n'
        '## Implementation Steps\n'
        '\n'
        '- [x] **1. Push the branch.**\n'
        '  - **Evidence:**\n'
        '  - **Notes:** no test was run.\n',
    )

    violations = check_plan(path, path.read_text(encoding='utf-8'))

    assert [(v.line, v.rule) for v in violations] == [
        (line_of(path, '1. Push the branch.'), RULE_EVIDENCE)
    ]


def test_evidence_may_wrap_onto_continuation_lines(plans_dir: Path) -> None:
    """Normalization wraps long claims, and a wrapped claim is still a claim."""
    path = write_plan(
        plans_dir,
        '20260101-wrapped-evidence',
        ACTIVE_FRONTMATTER,
        '# Wrapped evidence\n'
        '\n'
        '## Implementation Steps\n'
        '\n'
        '- [x] **1. Push the branch.**\n'
        '  - **Evidence:**\n'
        '    `git push` succeeded; CI green at `abc1234`.\n',
    )

    assert check_plan(path, path.read_text(encoding='utf-8')) == []


def test_unchecked_task_needs_no_evidence(plans_dir: Path) -> None:
    """An open task is honest by construction; only ticks make claims."""
    path = write_plan(
        plans_dir,
        '20260101-open-task',
        ACTIVE_FRONTMATTER,
        '# Open task\n\n## Implementation Steps\n\n- [ ] **1. Push the branch.**\n',
    )

    assert check_plan(path, path.read_text(encoding='utf-8')) == []


def test_done_plan_with_unchecked_task_is_a_violation(plans_dir: Path) -> None:
    """A shipped plan cannot still be carrying open work."""
    path = write_plan(
        plans_dir,
        '20260101-done-open',
        DONE_FRONTMATTER,
        '# Done with an open task\n'
        '\n'
        '## Implementation Steps\n'
        '\n'
        '- [x] **1. Land the code.**\n'
        '- [ ] **2. Push the branch.**\n',
    )

    violations = check_plan(path, path.read_text(encoding='utf-8'))

    assert [(v.line, v.rule) for v in violations] == [
        (line_of(path, '2. Push the branch.'), RULE_COMPLETENESS)
    ]


def test_done_plan_is_exempt_from_the_evidence_rule(plans_dir: Path) -> None:
    """Backfilling evidence into a closed plan would be inventing it."""
    path = write_plan(
        plans_dir,
        '20260101-done-no-evidence',
        DONE_FRONTMATTER,
        '# Closed record\n\n## Implementation Steps\n\n- [x] **1. Land the code.**\n',
    )

    assert check_plan(path, path.read_text(encoding='utf-8')) == []


def test_checkboxes_outside_implementation_steps_are_not_tasks(plans_dir: Path) -> None:
    """The format example under `## Approach` documents the rule, it is not a task."""
    path = write_plan(
        plans_dir,
        '20260101-approach-example',
        ACTIVE_FRONTMATTER,
        '# Approach example\n'
        '\n'
        '## Approach\n'
        '\n'
        '- [x] **3. Sync with `--locked`.** Illustrative, not a task.\n'
        '\n'
        '## Implementation Steps\n'
        '\n'
        '- [ ] **1. Push the branch.**\n',
    )

    assert check_plan(path, path.read_text(encoding='utf-8')) == []


def test_fenced_checkbox_inside_implementation_steps_is_ignored(plans_dir: Path) -> None:
    """A checkbox quoted in a code fence is documentation, not a claim."""
    path = write_plan(
        plans_dir,
        '20260101-fenced-example',
        ACTIVE_FRONTMATTER,
        '# Fenced example\n'
        '\n'
        '## Implementation Steps\n'
        '\n'
        '- [ ] **1. Document the format.** It looks like this:\n'
        '\n'
        '``` markdown\n'
        '- [x] **9. A ticked box with no evidence child.**\n'
        '```\n',
    )

    assert check_plan(path, path.read_text(encoding='utf-8')) == []


@pytest.mark.parametrize('heading', sorted(TASK_SECTIONS))
def test_both_task_section_headings_are_audited(plans_dir: Path, heading: str) -> None:
    """Plans predating `## Implementation Steps` use `## Tasks` and still count."""
    path = write_plan(
        plans_dir,
        '20260101-heading',
        ACTIVE_FRONTMATTER,
        f'# Heading variant\n\n## {heading}\n\n- [x] **1. Push the branch.**\n',
    )

    violations = check_plan(path, path.read_text(encoding='utf-8'))

    assert [(v.line, v.rule) for v in violations] == [
        (line_of(path, '1. Push the branch.'), RULE_EVIDENCE)
    ]


def test_every_violation_is_reported_not_just_the_first(plans_dir: Path) -> None:
    """A session fixing one breach must be able to see the rest."""
    path = write_plan(
        plans_dir,
        '20260101-many',
        ACTIVE_FRONTMATTER,
        '# Many breaches\n'
        '\n'
        '## Implementation Steps\n'
        '\n'
        '- [x] **1. First.**\n'
        '- [x] **2. Second.**\n'
        '  - **Evidence:** `pytest` passed.\n'
        '- [x] **3. Third.**\n',
    )

    violations = check_plan(path, path.read_text(encoding='utf-8'))

    assert [v.line for v in violations] == [
        line_of(path, '1. First.'),
        line_of(path, '3. Third.'),
    ]
    assert all(str(v).startswith(f'{path}:') for v in violations)


def test_check_directory_spans_every_plan(plans_dir: Path) -> None:
    """The gate runs over the directory, not one file a session remembered."""
    write_plan(
        plans_dir,
        '20260101-clean',
        ACTIVE_FRONTMATTER,
        '# Clean\n'
        '\n'
        '## Implementation Steps\n'
        '\n'
        '- [x] **1. Land it.**\n'
        '  - **Evidence:** `pytest` passed.\n',
    )
    write_plan(
        plans_dir,
        '20260102-dirty',
        ACTIVE_FRONTMATTER,
        '# Dirty\n\n## Implementation Steps\n\n- [x] **1. Land it.**\n',
    )

    violations = check_directory(plans_dir)

    assert [(v.path.name, v.rule) for v in violations] == [('20260102-dirty.md', RULE_EVIDENCE)]


def test_repository_plans_satisfy_the_checkbox_rules() -> None:
    """Every plan in this workspace obeys the rules it defines."""
    violations = check_directory(PLANS_DIR)

    assert not violations, 'plan checkbox violations:\n' + '\n'.join(
        str(violation) for violation in violations
    )
