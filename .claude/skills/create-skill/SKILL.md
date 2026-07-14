---
name: create-skill
description: Create, update, or review repository Agent Skills in .claude/skills with concise trigger descriptions, progressive-disclosure resources, optional Codex UI metadata, and validation. Use when adding or refining a SKILL.md, diagnosing skill discovery, or packaging a task-specific agent workflow.
---

# Create Agent Skills

Create focused, portable task playbooks. Keep only knowledge, procedures, and
resources that another capable agent would not reliably infer from the task or
the repository.

## Work in the Canonical Location

Create or update repository skills under `.claude/skills/<skill-name>/`.
`.codex/skills/` and `.cursor/skills/` are symlinks to that directory, so never
copy a skill into either of them. Use a personal skill directory only when the
user explicitly requests a user-wide skill.

For an existing skill, read its `SKILL.md` and every resource it directly
references before editing. Preserve its directory and frontmatter `name` unless
the user asks for a rename.

## Discover the Smallest Useful Scope

1. Extract the workflow, inputs, outputs, constraints, and corrections already
   present in the conversation or repository.
2. Identify several realistic requests that should trigger the skill and a few
   close requests that should not. Ask the user only about gaps that materially
   change the skill's scope or output.
3. Choose the narrowest useful workflow. Put general repository rules in
   `AGENTS.md`, not in a skill that would repeat them on every invocation.
4. Select the appropriate degree of prescription: explain heuristics when
   judgment varies; provide a parameterized pattern when a preferred approach
   exists; bundle and invoke a tested script when correctness depends on a
   repeatable, fragile sequence.

## Design for Progressive Disclosure

Keep the body concise—well below 500 lines whenever practical. Put the core
workflow and routing decisions in `SKILL.md`; add resources only when they
remove repeated work or substantial context.

| Resource | Add it when | Guidance |
| --- | --- | --- |
| `scripts/` | A deterministic operation would otherwise be rewritten or is error-prone. | Test the script, give it clear arguments and errors, and state exactly when to run it. |
| `references/` | Detailed schemas, policies, API material, or variant-specific procedures are needed. | Link directly from `SKILL.md` and say when to read it. Add a table of contents to long files. |
| `assets/` | A file is copied or used in the generated output rather than read as instructions. | Keep templates, images, fonts, and boilerplate here. |

Do not add README files, changelogs, quick-reference duplicates, placeholder
examples, or empty resource directories. Keep references one level from
`SKILL.md`; do not make an agent chase a chain of documents to begin work.

## Write `SKILL.md`

Use this minimal frontmatter exactly. Do not add platform-specific frontmatter
such as `allowed-tools`, `license`, or `compatibility`.

```yaml
---
name: example-skill
description: Create and validate example artifacts. Use when asked to generate, update, or check example artifacts for this repository.
---
```

Choose a lowercase hyphen-case name of at most 64 characters. Match the skill
directory name unless the repository has a deliberate naming convention that
requires otherwise.

Write the description as the discovery contract:

- State what the skill accomplishes and the concrete requests, files, or
  situations that should invoke it.
- Include natural alternate phrasings and useful adjacent cases, so the skill
  is not missed when the user does not name it directly.
- State meaningful boundaries when nearby work belongs to another skill.
- Prefer precise language to a keyword list. The body is not available during
  discovery, so do not rely on a `When to use` section to define triggers.

Write the body in imperative form. Explain the reason for consequential
guidance so the agent can adapt it correctly instead of following brittle,
overly rigid rules. Use a workflow layout for sequences, a task layout for
independent operations, and separate references for framework or domain
variants. Include examples, decision tables, or exact output structures only
when they make a recurring choice unambiguous.

Use relative Markdown links for bundled resources. Keep an instruction in one
place; link to its detailed explanation instead of duplicating it.

## Add Optional Codex UI Metadata

When the skill is intended for Codex's UI, add `agents/openai.yaml`. Keep it
outside `SKILL.md` frontmatter. Set a human-facing display name, a 25–64
character short description, and a one-sentence `default_prompt` that names
the skill as `$<skill-name>`. Include it in the independent Skill Creator
validation pass so it remains aligned with the skill.

Do not add icons, colors, MCP dependencies, or policy fields unless the user
has supplied the associated requirement or asset.

## Validate and Iterate

1. For a creation or substantial revision, delegate a read-only validation pass
   to a fresh subagent using the environment-provided `$skill-creator` skill.
   Give it the skill path and the task, but not an intended answer or diagnosis.
   Ask it to perform full validation, then report concrete corrections or that the capability
   is unavailable. Apply the relevant findings yourself.
2. Test bundled scripts with representative inputs. Use `./.tmp/` for
   temporary artifacts; never use `$TMPDIR`.
3. Forward-test a new, complex, or high-risk skill with realistic prompts when
   feasible. Give an independent evaluator only the task and relevant
   artifacts—not the intended answer or diagnosis. For an existing skill,
   compare the revision with a snapshot of the prior version when measuring an
   improvement.
4. Use objective checks for deterministic outcomes and human review for
   subjective quality. Remove instructions or resources that do not improve
   correctness, clarity, or efficiency.

## Definition of Done

- The description selects the skill for its intended requests without broadly
  colliding with adjacent skills.
- `SKILL.md` has only `name` and `description` in valid frontmatter.
- The body is concise, actionable, and routes detailed material to directly
  linked resources.
- Every bundled file has a purpose, and scripts have been exercised.
- Findings from proportionate independent validation are addressed, and any
  optional `agents/openai.yaml` matches the current skill.
