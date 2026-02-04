# PR #264 Description

Consolidate GitHub Copilot custom agents and enhance development guidance for AI agents working with the Dr.QP ROS 2 workspace.

## What Changed

Remove four redundant custom agent definitions that overlapped in functionality:
- `critical-thinking.agent.md` - functionality absorbed by mentor agent
- `implementation-plan.agent.md` - superseded by task-planner agent
- `plan.agent.md` - superseded by task-planner agent  
- `prompt-engineer.agent.md` - specialized functionality not frequently needed

Enhance the mentor agent with consolidated capabilities including critical thinking prompts, improved guidance on when to escalate to the principal-software-engineer, and clearer instructions for code review practices.

Migrate general development instructions from `.github/copilot-instructions.md` into the standardized `AGENTS.md` file, adding comprehensive ROS 2 build and test workflow guidance.

Update `refine-issue.agent.md` to reference the correct sub-agent after removing implementation-plan.

## Why These Changes

**Reduce Agent Confusion:** Multiple agents with overlapping purposes created ambiguity about which agent to use for planning, critical thinking, and issue refinement. Consolidating these into fewer, clearer agents improves the AI agent experience.

**Improve ROS 2 Development Workflow:** AI agents lacked clear guidance on incremental builds (`--packages-up-to`), targeted testing (`--packages-select`), devcontainer usage, and coverage collection. This led to inefficient full workspace builds and tests.

**Standardize Documentation:** Moving instructions to `AGENTS.md` aligns with project conventions and makes guidance discoverable in a single location.

## Impact

**For AI Agents:**
- Clearer agent selection with reduced overlap
- Faster iteration with incremental build/test guidance
- Better understanding of ROS 2 colcon workflow
- Proper devcontainer usage for remote development

**For Developers:**
- More maintainable agent definitions
- Single source of truth for development practices
- Clear code review guidelines (imperative mood, focus on intent over mechanics)

## Review Feedback Applied

Address all Copilot code review suggestions:
- Fix broken sub-agent reference in refine-issue.agent.md
- Restore code review guidance to emphasize neutral, code-focused feedback
- Clarify PR description guidelines for this project's workflow preferences
