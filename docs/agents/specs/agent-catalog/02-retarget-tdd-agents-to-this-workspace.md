# Spec 02: Retarget TDD agents to this workspace

- **Status**: proposed
- **Fixes**: AC1, AC2, AC3, AC4, AC5, AC12
- **Depends on**: 01 (soft — without it, keep `.codex/agents/` trampoline descriptions in
  sync by hand)
- **Files**: `.claude/agents/tdd-red.agent.md`, `.claude/agents/tdd-green.agent.md`,
  `.claude/agents/tdd-refactor.agent.md` (+ `.codex/agents/` trampolines if descriptions change)
- **Size**: M

## Objective

Rewrite the three TDD sub-agents so they describe _this_ workspace's stack and _actual_
sub-agent execution semantics: pytest/GTest under colcon, spec-or-issue-driven requirements,
autonomous operation reporting back to the orchestrator, and no outward-facing GitHub writes.

## Current behavior

The three files are recognizably a C#/.NET template dropped into a ROS 2 Python/C++ repo:

- **AC1 — wrong stack.** `tdd-red.agent.md` "C# Test Patterns": xUnit, FluentAssertions,
  AutoFixture, Theory tests; naming convention
  `Should_ReturnValidationError_When_EmailIsInvalid_Issue{number}` (pytest discovers `test_*`
  functions — tests named this way **would not run**). `tdd-green.agent.md` "C# Implementation
  Strategies": `List<T>`, `Dictionary<T,V>`. `tdd-refactor.agent.md`: NuGet vulnerability
  scanning, Azure Key Vault, `IOptions`, Serilog, nullable reference types, `Span<T>`/`Memory<T>`,
  SonarQube/Checkmarx, and a web-app OWASP checklist (XSS, SQL injection) — none of which exist
  here. This directly contradicts `AGENTS.md`: "**Always use `pytest`** — never `unittest`",
  ruff + `ament_flake8`, C++ Core Guidelines. Since sub-agents also load `CLAUDE.md`/`AGENTS.md`,
  every TDD run starts with two authoritative instruction sets in direct conflict.
- **AC2 — no runnable workflow.** None of the three mention `scripts/with-ros-env.sh`, `colcon
build/test`, `launch_pytest`, or the scaffolding skills (`add-test-file-python`,
  `add-test-file-cpp`, `launch-testing`, `ros2-workspace-testing`). "Verify the test fails —
  run the test" is unactionable as written: the agent must rediscover the build system from
  scratch on every invocation, or worse, follow the C# guidance.
- **AC3 — user-confirmation deadlock.** All three Execution Guidelines contain the identical
  step: "Confirm your plan with the user … NEVER start making changes without user
  confirmation." A sub-agent has no channel to the user (`create-agent` SKILL: it "does not see
  the parent conversation"; its final message goes to the orchestrator). Followed literally,
  the Principal Engineer's mandated Red→Green→Refactor orchestration stalls at every phase; in
  practice agents either ignore the rule or return without working.
- **AC4 — issue-number-from-branch heuristic.** TDD Red's only requirements source: "Extract
  issue number from branch name pattern `*{number}*`" then `gh issue view`/`--search`. On
  spec-program branches (`locomotion-spec-02-time-based-gait-targets`) the first match is the
  **spec** number and resolves to an unrelated or missing issue. Meanwhile
  `docs/agents/specs/README.md` names these very agents as the workflow ("TDD Red → TDD Green
  → TDD Refactor … Every spec lists the tests to write first") — yet no agent knows specs exist.
- **AC5 — inverted caution.** TDD Green: "Update issue with implementation progress"; TDD
  Refactor: "Comment on final implementation and **close issue** if complete." Leaf sub-agents
  performing ungated outward-facing GitHub writes, while AC3 forbids them ungated _local file
  edits_.
- **AC12 — tool over-grant.** All three carry `Agent` in `tools:` despite being terminal
  workers; `create-agent` prescribes least privilege and warns against over-orchestration.

## Target design

Rewrite each agent (keep them short — every line is paid for per invocation):

1. **Requirements source (Red).** Accept, in priority order: (a) a spec file path passed in the
   orchestrator prompt (the `docs/agents/specs/` programs — use its "Test plan (write first)"
   section verbatim), (b) an explicit issue number passed in the prompt, (c) _fallback only_:
   the branch-name heuristic, with the instruction to verify the fetched issue's title/body
   actually matches the task before trusting it.
2. **Stack sections.** Replace the C# sections with pointers, not prose: Python tests →
   `add-test-file-python` + `launch-testing` skills, pytest naming (`test_*`), AAA structure;
   C++ tests → `add-test-file-cpp` (GTest/GMock); build/run →
   `scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to <pkg>` and
   `colcon test --packages-select <pkg>`, results from `log/latest_test/<pkg>/`. Refactor
   phase: ruff via `scripts/python-reformat.sh`, `ament_flake8` gate via
   `scripts/python-lint-check.sh` (the `python-format-lint` skill), and the `AGENTS.md`
   exception-handling rules — drop the OWASP/Azure/Serilog material entirely; robot-relevant
   hardening (input validation on external inputs, no secrets in code) can stay as two lines.
3. **Interaction model.** Replace every "confirm with the user" step with: state assumptions
   and open questions **in the final report to the orchestrator**; if requirements are too
   ambiguous to write a failing test, return early with the questions instead of guessing.
   Remove issue commenting/closing; the report lists suggested issue updates for the
   orchestrator (or user) to act on. This matches the Principal Engineer's existing "check
   results before proceeding" orchestration contract.
4. **Tools.** Drop `Agent` from all three (`Bash, Read, Edit, Write, Grep, Glob`).
5. Keep what is genuinely good and stack-neutral: one-test-at-a-time, fail-for-the-right-reason,
   minimal-implementation, small-steps-with-green-tests, and the phase checklists (rewritten to
   drop issue-closing and C# items). Normalize spelling to match the rest of the repo while
   rewriting.

## Out of scope

- Principal Engineer edits (spec 03) — but its TDD orchestration section must remain accurate;
  if phase names/contracts change, adjust the cross-references there in the same PR as a
  mechanical follow-through.
- Adding new agents (e.g. the `task-researcher`/`task-planner` referenced by
  `pr-feedback-resolution` — spec 04 removes those references instead).

## Test plan (write first)

- `validate_agent_files --recommend .claude` passes on the rewritten files (frontmatter,
  description length, link resolution).
- Grep-level regression checks (add to the validator's tests or a small doc-lint test):
  no occurrence of `xUnit|FluentAssertions|AutoFixture|NuGet|Serilog|Key Vault|IOptions|Span<T>|SonarQube|Checkmarx` under `.claude/agents/`;
  no occurrence of `[Cc]onfirm .* with the user` under `.claude/agents/tdd-*`.
- Relative links to `../skills/<name>/` resolve (validator already checks links).

## Verification

Representative dry runs (no better substitute exists for prompt changes):

1. From a spec branch, invoke `TDD Red` with a locomotion spec path in the prompt → it derives
   tests from the spec's test plan, writes **one** failing pytest test, runs it via
   `scripts/with-ros-env.sh colcon test --packages-select <pkg>`, and reports back without
   asking the user anything.
2. Invoke `TDD Green` on that failing test → minimal implementation, all tests green, no
   `gh issue` writes in the transcript.
3. `TDD Refactor` → runs ruff/`ament_flake8` scripts, tests stay green.

## Acceptance criteria

- [ ] Zero C#/.NET tooling references in the three agent files; Python/C++ guidance defers to
      the existing skills rather than restating them.
- [ ] Red phase consumes `docs/agents/specs/` spec files as a first-class requirements source;
      branch-digit heuristic demoted to verified fallback.
- [ ] Build/test instructions go through `scripts/with-ros-env.sh` + colcon selectors.
- [ ] No "confirm with the user" steps; ambiguity is returned to the orchestrator.
- [ ] No autonomous GitHub issue writes; `Agent` removed from all three `tools:` lists.
- [ ] Codex trampoline descriptions in sync (validated automatically once spec 01 lands);
      `validate_agent_files` passes.
