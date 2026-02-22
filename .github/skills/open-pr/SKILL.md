---
name: open-pr
description: Create GitHub pull requests from conversation context with proper formatting and tag selection
---

# Open PR

This skill instructs AI agents on how to create GitHub pull requests from conversation context
with meaningful titles, proper formatting, and appropriate tag selection. The AI agent
should analyze the conversation, extract PR details, and confirm with the user before
creating the pull request.

## GitHub MCP Tools Required

This skill **MUST** use GitHub MCP tools for GitHub operations and **MUST NOT** use `gh` CLI commands.

Required MCP tools:
- `github/create_pull_request` - create the pull request
- `github/list_issues` - list recent issues when issue number is missing

Optional MCP tools (for validation and follow-up):
- `github/list_pull_requests` - check for existing PRs from the same branch
- `github/pull_request_read` - fetch PR details after creation

## PR Description Source of Truth

The PR body content **MUST** be generated using the prompt in
`.github/prompts/generate-pr-description.prompt.md`.

The open-pr skill is responsible for:
- tag selection for the PR title
- issue-number enforcement (`[tag][#N]`)
- user confirmation
- remote branch verification
- GitHub PR creation through MCP

The detailed PR description structure, section requirements, and quality checks
are defined in `.github/prompts/generate-pr-description.prompt.md` and **MUST NOT**
be duplicated here.

## Tag Selection

A `git-msg-tags.md` file should appear in `{ROOT_PROJ}/docs/git-msg-tags.md` which
defines the tags related to the corresponding modules or modifications. The AI agent
**MUST** refer to this file to select the appropriate tag for the PR title.

If the file does not exist, reject the PR creation and ask the user to provide a
list of tags in `docs/git-msg-tags.md`.

### Tag Logic

The AI agent must determine which tag to use based on the PR type by reading
`docs/git-msg-tags.md` which contains the project's tag definitions.

**Selection guidelines:**
- Read `docs/git-msg-tags.md` to understand available tags and their meanings
- Choose the most specific tag that describes the primary change
- If multiple tags could apply, choose the one that best represents the core purpose
- If the tag is ambiguous, ask the user to select from 2-3 most relevant options

## Workflow for AI Agents

When this skill is invoked, the AI agent **MUST** follow these steps:

### 1. Context Analysis Phase

Review the entire conversation history and git changes to extract PR details:
- Identify what work was completed during the conversation
- Review git diff and git status to see actual changes made
- Extract key details: what was changed, why, which files were affected
- Determine the type of changes (feature, bugfix, refactor, etc.)
- Check if there's a related issue number mentioned in the conversation

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

### 3. Tag Selection Phase

- Read `docs/git-msg-tags.md` to understand available tags
- Analyze the changes and determine the primary purpose
- Apply the tag logic described above
- If multiple tags could apply, choose the most specific one
- If the tag is ambiguous, ask the user to choose from 2-3 most relevant options

### 4. Issue Number Extraction

**CRITICAL:** The PR title **MUST** include an issue number in the format `[tag][#N]`.

**How to find the issue number:**
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
   - Ask the user: "Which issue does this PR address? (Provide issue number)"

3. If user says there's no related issue:
   - **STOP** and inform the user:
     ```
     Cannot create PR without a related issue.
     Please create an issue first using the open-issue skill, or provide an existing issue number.
     ```

**Never create a PR without an issue number.**

### 5. PR Draft Construction

Generate the PR description by following
`.github/prompts/generate-pr-description.prompt.md`.

Use the generated output as the PR body, and prepend the required title format:
- `[tag][#issue-number] Brief description`
- Keep the title description concise and outcome-focused

### 6. User Confirmation Phase

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

### 6.5. Remote Branch Verification

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

### 7. GitHub PR Creation (MCP)

Once confirmed and the branch is on remote, create the PR using `github/create_pull_request`.

Use the tool with these fields:
- `owner` (required): repository owner
- `repo` (required): repository name
- `title` (required): full PR title
- `head` (required): source branch name
- `base` (required): target branch (usually `main`)
- `body` (optional): PR body (include Summary, Changes, Testing, Related Issue)
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

### 8. Error Handling

Handle common error scenarios gracefully:

**Missing git-msg-tags.md:**
```
Cannot create PR: docs/git-msg-tags.md not found.
Please create this file with your project's tag definitions.
```

**No issue number found:**
```
Cannot create PR: No related issue number found.

Please either:
1. Provide the issue number this PR addresses
2. Create an issue first using the open-issue skill
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
- What issue does this PR address?
- What was tested?
```

**PR creation failed:**
```
Failed to create pull request: [error message]
Please check GitHub MCP connectivity, authentication, and required tool permissions.
```

## Ownership

The AI agent **SHALL NOT** claim authorship or co-authorship of the pull request.
The PR is created on behalf of the user, who is **FULLY** responsible for its content.

Do not add any "Created by AI" or similar attributions to the PR body unless
explicitly requested by the user.

## PR Body Guidance

For complete PR-description instructions and examples, always use:
`.github/prompts/generate-pr-description.prompt.md`.
