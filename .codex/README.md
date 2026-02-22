# Codex Customizations

This directory mirrors the repository's GitHub Copilot customizations so they
are discoverable in Codex.

Source of truth:
- .github/agents
- .github/skills
- .github/instructions
- .github/prompts

Codex symlinks:
- .codex/agents -> .github/agents
- .codex/skills -> .github/skills
- .codex/instructions -> .github/instructions
- .codex/prompts -> .github/prompts

Update the .github sources; the symlinks pick up changes automatically.
