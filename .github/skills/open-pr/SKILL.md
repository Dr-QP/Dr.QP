---
name: open-pr
description: Create GitHub pull requests from conversation context with proper formatting and optional issue linking
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

The PR body content **MUST** be generated using the prompt in
`.github/prompts/generate-pr-description.prompt.md`.

The open-pr skill is responsible for:
- optional issue linking in title/body when issue context exists
- user confirmation
- delegating branch sync with `origin/main` to the `update-branch` skill
- remote branch verification
- GitHub PR creation through MCP

The detailed PR description structure, section requirements, and quality checks
are defined in `.github/prompts/generate-pr-description.prompt.md` and **MUST NOT**
be duplicated here.

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
# Check what files have changed
git status

# Review the actual changes
git diff

# Check commit history on current branch
git log origin/main..HEAD --oneline
```

This ensures the PR description accurately reflects the actual code changes.

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

Generate the PR description by following
`.github/prompts/generate-pr-description.prompt.md`.

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
`.github/skills/update-branch/SKILL.md` workflow.

The AI agent **MUST** invoke and follow the `update-branch` skill instead of re-implementing
merge logic inline.

If `update-branch` reports unresolved conflicts or requires user input, stop PR creation and
ask the user to resolve or confirm conflict decisions first.

### 7. Remote Branch Verification

**CRITICAL:** Before creating the PR, verify the current branch exists on the remote repository.

Check if the current branch is tracking a remote branch:

```bash
# Check if current branch has an upstream branch
git rev-parse --abbrev-ref --symbolic-full-name @{u} 2>/dev/null
```

**If the command fails (no upstream branch):**
1. Get the current branch name:
   ```bash
   git branch --show-current
   ```
2. Push the branch with tracking:
   ```bash
   git push -u origin <branch-name>
   ```
3. Confirm to user: "Pushed branch to remote: origin/<branch-name>"

**If the command succeeds (upstream branch exists):**
1. Check if local is ahead of remote:
   ```bash
   git status --porcelain --branch
   ```
2. If output contains `[ahead N]`, push changes:
   ```bash
   git push
   ```
3. If up-to-date, continue to PR creation

**Error handling:**
- If push fails due to authentication:
  ```
  Git push failed. Please check your Git credentials.
  ```
- If push fails due to conflicts:
  ```
  Cannot push: your branch has diverged from remote.
  Please resolve conflicts manually with:
    git pull --rebase origin <branch-name>
  ```
- For other push failures: Display the error and abort PR creation

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
- The body should be the generated markdown from
  `.github/prompts/generate-pr-description.prompt.md`
- Do not duplicate or re-interpret the prompt's section requirements here
- If `base` is not explicitly provided by user/repo policy, default to `main`
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

For complete PR-description instructions and examples, always use:
`.github/prompts/generate-pr-description.prompt.md`.
