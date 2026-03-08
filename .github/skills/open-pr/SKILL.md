---
name: open-pr
description: 'Create GitHub pull requests from conversation context with accurate title/body generation, user confirmation, branch sync, and remote verification. Use when asked to open/create/submit a PR, draft a pull request, or finalize changes after implementation. Keywords: open pr, create pr, submit pr, pull request, github pr, draft pr, ready for review.'
---

# Open PR

This skill instructs AI agents on how to create GitHub pull requests from conversation context
with meaningful titles and proper formatting. The AI agent
should analyze the conversation, extract PR details, and confirm with the user before
creating the pull request.

## GitHub MCP Tools Required

This skill **MUST** use GitHub MCP tools for GitHub operations and **MUST NOT** use `gh` CLI commands.

Required MCP tools:

- `github/create_pull_request` - create the pull request

Optional MCP tools (for validation and follow-up):

- `github/list_issues` - list recent issues when issue number is missing
- `github/list_pull_requests` - check for existing PRs from the same branch
- `github/pull_request_read` - fetch PR details after creation

## PR Description Source of Truth

The PR body content **MUST** be generated using the
[generate-pr-description](../generate-pr-description/) skill.

The open-pr skill is responsible for:

- optional issue linking in title/body when issue context exists
- user confirmation
- delegating branch sync with `origin/main` to the `update-branch` skill
- remote branch verification
- GitHub PR creation through MCP

## Bundled Scripts

Use these exact helper scripts instead of retyping inline shell commands:

- [review-git-changes.sh](scripts/review-git-changes.sh) reviews the working tree, diff stats, patch, and commit log for the PR context.
- [ensure-remote-branch.sh](scripts/ensure-remote-branch.sh) verifies upstream tracking, pushes the branch when needed, and blocks on divergence.

The detailed PR description structure, section requirements, and quality checks
are defined in the [generate-pr-description](../generate-pr-description/) skill
and **MUST NOT** be duplicated here.

## Workflow for AI Agents

When this skill is invoked, the AI agent **MUST** follow these steps:

### 1. Context Analysis Phase

Review the entire conversation history and git changes to extract PR details:

- Identify what work was completed during the conversation
- Review git diff and git status to see actual changes made
- Extract key details: what was changed, why, which files were affected
- Determine the type of changes (feature, bugfix, refactor, etc.)
- Check if there's a related issue number mentioned in the conversation (optional)

Context signals for PR type:

- Feature signals: new functionality added, new files created, capabilities extended
- Bugfix signals: fixed error, resolved issue, corrected behavior
- Refactor signals: improved code structure, reorganized code, better patterns
- Documentation signals: updated README, added comments, wrote guides
- Test signals: added test coverage, modified test cases

### 2. Git Changes Review

**CRITICAL:** Before drafting the PR, the AI agent **MUST** review actual git changes:

```bash
.github/skills/open-pr/scripts/review-git-changes.sh
```

Use `--base-ref <ref>` or `--range <range>` when the comparison base is not `origin/main`.
This script ensures the PR description accurately reflects the actual code changes.

### 3. Optional Issue Linking

Issue linking is recommended but not required.

**How to find an issue number when available:**

1. Search conversation history for explicit issue references:
   - "for issue #42"
   - "closes #15"
   - "related to #23"
   - GitHub issue URLs containing issue numbers

2. If no issue number is found in conversation:
   - Check if there are recent issues that match this work:
     - Use `github/list_issues` with repository `owner` and `repo`
     - Start with `state: open`, `perPage: 10`
     - If needed, broaden query with `state: all`

- Ask the user if they want to link an issue: "Would you like to link an issue to this PR?"

3. If no issue is provided:

- Continue PR creation without issue linking
- Use a concise title without issue prefix

### 4. PR Draft Construction

Generate the PR description by following the
[generate-pr-description](../generate-pr-description/) skill.

Use the generated output as the PR body, and use one of these title formats:

- If issue is available: `[#issue-number] Brief description`
- If issue is not available: `Brief description`
- Keep the title description concise and outcome-focused

### 5. User Confirmation Phase

**CRITICAL:** The AI agent **MUST** display the complete PR draft to the user
and wait for explicit confirmation before creating the PR.

Present the draft in a clear format:

```
I've prepared this pull request:

---
[Full PR content here]
---

Should I create this PR?
```

- Wait for explicit "yes", "confirm", "create it", or similar affirmative response
- If the user requests modifications, update the draft and present again
- If the user declines, abort PR creation gracefully

### 6. Branch Sync with `origin/main`

**CRITICAL:** Before pushing or creating a PR, sync the current branch using the
`.github/skills/update-branch/` workflow.

The AI agent **MUST** invoke and follow the `update-branch` skill instead of re-implementing
merge logic inline.

If `update-branch` reports unresolved conflicts or requires user input, stop PR creation and
ask the user to resolve or confirm conflict decisions first.

### 7. Remote Branch Verification

**CRITICAL:** Before creating the PR, verify the current branch exists on the remote repository.

Run the bundled helper:

```bash
.github/skills/open-pr/scripts/ensure-remote-branch.sh
```

The script handles these cases exactly:

- no upstream branch: pushes with `-u origin <branch>`
- local branch ahead of upstream: pushes changes
- branch up to date: exits successfully without pushing
- branch behind or diverged: exits non-zero with the exact recovery instructions to show the user
- current branch is `main` or `master`: exits with a warning so the agent can ask for confirmation before continuing

Use `--remote <name>` or `--branch <name>` when the default remote or branch should be overridden.

If the script exits non-zero, display its actionable error output and abort PR creation.

### 8. GitHub PR Creation (MCP)

Once confirmed and the branch is on remote, create the PR using `github/create_pull_request`.

Use the tool with these fields:

- `owner` (required): repository owner
- `repo` (required): repository name
- `title` (required): full PR title
- `head` (required): source branch name
- `base` (required): target branch (usually `main`)
- `body` (optional): PR body (include Summary, Changes, Testing, optional Related section)
- `draft` (optional): set to `true` for draft PR
- `maintainer_can_modify` (optional): set per repository policy

**Important:**

- The body should be the generated markdown from the
  [generate-pr-description](../generate-pr-description/) skill
- Do not duplicate or re-interpret the prompt's section requirements here
- If `base` is not explicitly provided by user/repo policy, set `base: main`.
- After successful creation, display the PR URL/number returned by the MCP tool
- Confirm: "Pull request created successfully: [URL]"

**Optional parameters:**

- Set `draft: true` if the user wants to create a draft PR
- Set `base: <branch>` if targeting a different base branch

### 9. Error Handling

Handle common error scenarios gracefully:

**Issue number not found:**

```
No related issue number found.
Proceeding without issue linking.
```

**No git changes:**

```
Cannot create PR: No changes detected in the working directory.
Please make and commit your changes first.
```

**GitHub MCP authentication/authorization failure:**

```
GitHub MCP request failed due to authentication or missing permissions.
Please verify MCP server authentication and token scopes (typically `repo`).
```

**Not on a feature branch:**

```
Warning: You're on the main/master branch.
PRs should typically be created from feature branches.

Create a new branch with:
  git checkout -b feature/your-feature-name

Or confirm you want to create a PR from the current branch.
```

**No conversation context:**

```
I don't have enough context to create a PR. Could you please provide:
- What changes were made?
- What was tested?
```

**PR creation failed:**

```
Failed to create pull request: [error message]
Please check GitHub MCP connectivity, authentication, and required tool permissions.
```

**Merge conflict while syncing with `origin/main`:**

```
Cannot continue PR creation: merge conflicts occurred while merging origin/main.
Please resolve conflicts, commit the merge, and retry PR creation.
```

## Ownership

The AI agent **SHALL NOT** claim authorship or co-authorship of the pull request.
The PR is created on behalf of the user, who is **FULLY** responsible for its content.

Do not add any "Created by AI" or similar attributions to the PR body unless
explicitly requested by the user.

## PR Body Guidance

For complete PR-description instructions and examples, use the
[generate-pr-description](../generate-pr-description/) skill.
