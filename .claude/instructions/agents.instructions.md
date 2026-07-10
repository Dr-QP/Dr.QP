---
description: 'Guidelines for creating custom agent files for Claude Code'
applyTo: '**/*.agent.md'
---

# Custom Agent File Guidelines

Instructions for creating effective and maintainable custom agents (subagents) that provide specialized expertise for specific development tasks in Claude Code.

## Project Context

- Target audience: Developers creating custom agents for Claude Code
- File format: Markdown with YAML frontmatter
- File naming convention: lowercase with hyphens, `.agent.md` suffix (e.g., `test-specialist.agent.md`)
- Location: `.claude/agents/` (project-level, canonical in this repository), `~/.claude/agents/` (personal, user-wide)
- Purpose: Define specialized agents with tailored expertise, tools, and instructions for specific tasks
- Official documentation: https://code.claude.com/docs/en/sub-agents

Claude Code discovers every `*.md` file in `.claude/agents/`; this repository uses the `.agent.md` suffix so agent specs are easy to distinguish and validate. Codex consumes the same specs through minimal trampoline files in `.codex/agents/` (see "Codex Trampolines" below). Cursor reads them via the `.cursor/agents/` symlink.

## Required Frontmatter

Every agent file must include YAML frontmatter with the following fields:

```yaml
---
name: Agent Display Name
description: Brief description of the agent purpose and when to delegate to it
tools: Bash, Read, Edit, Write, Grep, Glob
model: inherit
---
```

### Core Frontmatter Properties

#### **name** (REQUIRED)

- Identifier for the agent; shown in the UI and used when invoking the agent via the Agent tool
- Use title case and be descriptive
- Example: `Testing Specialist`

#### **description** (REQUIRED)

- Clearly states the agent's purpose, domain expertise, and **when Claude should delegate to it**
- Claude uses this description to decide when to hand a task to the agent automatically, so include trigger scenarios ("Use during the Red phase of TDD — before any implementation exists")
- Should be concise (50-250 characters) and actionable

#### **tools** (OPTIONAL)

- Comma-separated string (or YAML list) of Claude Code tool names the agent can use
- If omitted, the agent inherits all tools available to the main conversation
- Common tools: `Bash`, `Read`, `Edit`, `Write`, `Grep`, `Glob`, `WebSearch`, `WebFetch`, `Agent`, `TodoWrite`
- MCP tools use the `mcp__<server>__<tool>` naming scheme

#### **model** (OPTIONAL)

- Model the agent runs on: `sonnet`, `opus`, `haiku`, or `inherit` (use the parent conversation's model)
- If omitted, the configured subagent default is used
- Choose based on agent complexity: `haiku` for mechanical tasks, `sonnet`/`opus` for judgment-heavy work

### Tool Selection Best Practices

- **Principle of Least Privilege**: Only enable tools necessary for the agent's purpose
- **Security**: Limit `Bash` access unless command execution is explicitly required
- **Focus**: Fewer tools = clearer agent purpose and better performance
- **Read-only agents**: Reviewers and analysts should get `Read`, `Grep`, `Glob` — not `Edit`/`Write`

## Agent Prompt Structure

The markdown content below the frontmatter defines the agent's behavior, expertise, and instructions. Well-structured prompts typically include:

1. **Agent Identity and Role**: Who the agent is and its primary role
2. **Core Responsibilities**: What specific tasks the agent performs
3. **Approach and Methodology**: How the agent works to accomplish tasks
4. **Guidelines and Constraints**: What to do/avoid and quality standards
5. **Output Expectations**: Expected output format and quality

### Prompt Writing Best Practices

- **Be Specific and Direct**: Use imperative mood ("Analyze", "Generate"); avoid vague terms
- **Define Boundaries**: Clearly state scope limits and constraints
- **Include Context**: Explain domain expertise and reference relevant frameworks
- **Focus on Behavior**: Describe how the agent should think and work
- **Use Structured Format**: Headers, bullets, and lists make prompts scannable
- **Remember the agent starts cold**: A subagent does not see the parent conversation; the prompt (plus what the orchestrator passes in) is all the context it gets

## Sub-Agent Orchestration

Agents can invoke other agents using the **Agent tool** to orchestrate multi-step workflows (e.g., the Principal Engineer agent orchestrating the TDD Red → Green → Refactor cycle).

### How It Works

1. Include `Agent` in the orchestrator's `tools` list
2. For each step, invoke a sub-agent with `subagent_type` set to the target agent's name and a `prompt` carrying the essential context
3. The sub-agent's final message is returned to the orchestrator — ask for a clear summary of actions taken, files modified, and issues found

### Orchestration Guidelines

- **Pass minimal, explicit context**: Paths, identifiers, and expected outputs — not entire file contents
- **Sequential execution**: Run steps in order when outputs feed later steps; independent steps can run in parallel
- **Return summaries**: Each sub-agent should report what it accomplished
- **Error handling**: Check results before proceeding to dependent steps
- **Don't over-orchestrate**: Each sub-agent invocation adds latency and re-derives context. Avoid pipelines of more than a handful of steps or bulk data processing through sub-agents; implement high-volume logic directly instead

### Example Step Invocation

```text
Step 1: Write failing tests
Agent: TDD Red
Prompt: Work on issue #123 in packages/runtime/drqp_brain.
        Write failing tests for the gait-selection behaviour described in the issue.
        Return: test files created, how each test fails, open questions.
```

## Codex Trampolines

Codex discovers agents through `.codex/agents/*.md`. Each trampoline is a minimal wrapper that delegates to the canonical Claude spec:

```markdown
---
name: TDD Red
description: <same description as the canonical agent>
---

Read and follow all instructions in `.claude/agents/tdd-red.agent.md`, adapting tool names to the Codex environment.
```

When creating or renaming an agent, add or update the matching trampoline. Keep `name` and `description` in sync with the canonical file; everything else lives only in `.claude/agents/`.

## Agent Creation Checklist

### Frontmatter

- [ ] `name` present and descriptive
- [ ] `description` states what the agent does **and when to delegate to it**
- [ ] `tools` limited to what the agent needs (or intentionally omitted to inherit all)
- [ ] `model` set when the default is not appropriate

### Prompt Content

- [ ] Clear agent identity and role defined
- [ ] Core responsibilities listed explicitly
- [ ] Approach and methodology explained
- [ ] Guidelines and constraints specified
- [ ] Output expectations documented
- [ ] Instructions are specific and actionable
- [ ] Scope and boundaries clearly defined

### File Structure

- [ ] Filename follows lowercase-with-hyphens convention with `.agent.md` suffix
- [ ] File placed in `.claude/agents/`
- [ ] Matching Codex trampoline exists in `.codex/agents/`
- [ ] Relative links to skills (`../skills/<name>/`) and sibling agents resolve

### Quality Assurance

- [ ] Agent purpose is unique and not duplicative
- [ ] Tools are minimal and necessary
- [ ] Agent has been tested with representative tasks
- [ ] `validate_agent_files` passes (run by pre-commit and CI)

## Common Mistakes to Avoid

- ❌ Missing `description`, or a description that says _what_ but not _when_ — the agent will never be selected automatically
- ❌ Granting all tools to a read-only reviewer agent
- ❌ Referencing tools that don't exist in Claude Code (tool aliases from other assistants)
- ❌ Assuming the sub-agent can see the parent conversation — it can't; pass context explicitly
- ❌ Editing a Codex trampoline instead of the canonical `.claude/agents/` spec
- ❌ Vague, ambiguous instructions or conflicting guidelines
- ❌ Using spaces or special characters in filenames

## Testing and Validation

1. Create the agent file with proper frontmatter
2. Run `validate_agent_files --recommend .claude` (also enforced by pre-commit and CI)
3. Start a Claude Code session and confirm the agent appears (`/agents`)
4. Test with representative tasks: explicit invocation ("Use the TDD Red agent to…") and automatic delegation
5. Verify tool access works as expected and the agent stays within scope

## Additional Resources

- [Claude Code subagents documentation](https://code.claude.com/docs/en/sub-agents)
- [Agent Skills guidelines](./agent-skills.instructions.md) — for creating skills
- Existing agents in `.claude/agents/` — use as reference implementations
