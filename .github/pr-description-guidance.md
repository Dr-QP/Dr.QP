# Pull request description guidance

Repository-specific instructions for the `agentdev:pr-gen-description` skill.
The skill owns the section structure; this file adds what that structure does
not already say. Captured from this repository's previous
`.github/pull_request_template.md` when the template was reduced to a stub.

These are instructions, not headings — do not treat them as a section list.

- In **What Changed**, group related changes by behavior or feature area
  instead of listing files.
- In **Verification**, note coverage impact and any important gaps when the
  change affects a package with coverage enabled. This workspace builds
  coverage with `--cmake-args -DDRQP_ENABLE_COVERAGE=ON` and the
  `coverage-pytest` mixin, so a coverage delta is usually available.
- Omit sections that do not apply rather than emitting them with placeholder
  or "None." content.

The **Verification** / **Reviewer Handoff** split is fixed: Verification records
what was already run, Reviewer Handoff records what the reviewer still needs to
do. Never merge or rename those two sections.
