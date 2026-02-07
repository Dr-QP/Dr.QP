# Cursor Customizations

This directory mirrors the repository's GitHub Copilot customizations so they
are discoverable in Cursor.

Source of truth:
- .github/agents
- .github/skills
- .github/instructions

Cursor symlinks:
- .cursor/agents -> .github/agents
- .cursor/skills -> .github/skills
- .cursor/rules -> .github/instructions

Update the .github sources; the symlinks pick up changes automatically.
