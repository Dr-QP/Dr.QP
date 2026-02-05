---
description: 'Generate comprehensive pull request description following code-review-standards with change analysis, testing strategy, and migration notes'
name: 'generate-pr-description'
agent: 'agent'
tools: ['read', 'edit', 'search']
argument-hint: '[--branch branch-name] [--commits commit-range]'
---

# Generate Pull Request Description

Generate a comprehensive, high-quality pull request description that follows code-review-standards skill and engineering best practices.

## When to Use This Prompt

- Creating a pull request for code changes
- Need detailed PR description that explains changes
- Want to follow project conventions for PR documentation
- Preparing for code review
- Documenting technical decisions and assumptions

## Prerequisites

- Changes are committed to a branch
- Understanding of what was changed and why
- Related GitHub issues are identified
- Testing has been performed

## Inputs

### Required Inputs

One of the following:
- **Current Branch**: Use current git branch (default)
- **Branch Name** `${input:branchName}`: Specific branch to analyze
- **Commit Range** `${input:commitRange}`: Specific commit range (e.g., `main..feature-branch`)

### Optional Inputs

- **PR Title**: Generated from branch name or first commit, can override
- **Related Issues**: GitHub issue numbers (e.g., `#123, #456`)
- **Breaking Changes**: Yes/No flag
- **Migration Steps**: Required migration or deployment steps

## Workflow

### Step 1: Analyze Git Changes

1. Determine what to analyze:
   - If branch name provided: `git diff main..<branch_name>`
   - If commit range provided: `git diff <commit_range>`
   - Otherwise: `git diff main..HEAD`

2. Collect comprehensive change information:
   ```bash
   # Get list of changed files
   git diff --name-status <range>
   
   # Get detailed diff
   git diff <range>
   
   # Get commit messages
   git log --oneline <range>
   
   # Get statistics
   git diff --stat <range>
   ```

3. Categorize changes:
   - New files added
   - Files modified
   - Files deleted
   - Files renamed/moved

### Step 2: Identify Change Categories

Categorize all changes into:

- **Features**: New functionality added
- **Bug Fixes**: Defects resolved
- **Refactoring**: Code improvements without behavior change
- **Tests**: Test additions or modifications
- **Documentation**: README, comments, docs changes
- **Configuration**: Build config, CI/CD, dependencies
- **Performance**: Optimizations
- **Security**: Security improvements or fixes

### Step 3: Extract Technical Details

For each significant change:

1. **Purpose**: Why was this change made?
2. **Approach**: How was it implemented?
3. **Files affected**: Which files were changed?
4. **Dependencies**: New dependencies or version changes?
5. **Side effects**: Unintended consequences or related impacts?

### Step 4: Identify Related Issues

1. Search commit messages for issue references (#123, GH-456, etc.)
2. Search branch name for issue numbers
3. Prompt user to confirm or add additional issues
4. Link to:
   - GitHub Issues
   - Design documents
   - Spike research
   - Related PRs

### Step 5: Assess Testing Strategy

Document testing performed:

- **Unit Tests**: New or modified unit tests
- **Integration Tests**: Integration test coverage
- **Manual Testing**: Steps performed manually
- **Test Results**: Summary of test outcomes
- **Coverage**: Impact on code coverage

### Step 6: Check for Breaking Changes

Identify breaking changes:

- API changes (method signatures, parameters)
- Configuration changes
- Dependency version changes
- Database schema changes
- File format changes
- Protocol changes

### Step 7: Generate PR Description

Using code-review-standards skill template:

```markdown
## Summary

<!-- High-level overview of the PR in 2-3 sentences -->

## Changes

### Features
- Feature 1: Description
- Feature 2: Description

### Bug Fixes
- Fix 1: Description and root cause
- Fix 2: Description and root cause

### Refactoring
- Refactoring 1: What and why
- Refactoring 2: What and why

### Tests
- Test additions/modifications

### Documentation
- Documentation updates

### Configuration
- Build/CI/CD changes

## Technical Details

### Approach

<!-- Explain the technical approach taken -->

### Key Files Changed

- `path/to/file1.cpp`: Purpose of changes
- `path/to/file2.py`: Purpose of changes
- `path/to/file3.md`: Purpose of changes

### Dependencies

<!-- List any new or updated dependencies -->
- Added: `<dependency>` v<version> - <reason>
- Updated: `<dependency>` <old_version> → <new_version> - <reason>

### Assumptions

<!-- Explicitly document any assumptions made -->

1. Assumption 1
2. Assumption 2

## Testing Strategy

### Unit Tests

- [ ] All existing tests pass
- [ ] New tests added for new functionality
- [ ] Edge cases covered

### Integration Tests

- [ ] Integration tests updated/added
- [ ] All integration tests pass

### Manual Testing

Steps performed:
1. Step 1
2. Step 2
3. Step 3

Results: <!-- Describe results -->

### Coverage

- Overall coverage: <percentage>%
- New code coverage: <percentage>%
- Files with < 80% coverage: <list>

## Breaking Changes

<!-- YES or NO -->

### Migration Steps

<!-- If breaking changes exist, provide migration guide -->

1. Migration step 1
2. Migration step 2

### Deployment Notes

<!-- Special deployment considerations -->

## Risk Assessment

### Risks

- Risk 1: Description and impact
- Risk 2: Description and impact

### Mitigation

- Mitigation strategy 1
- Mitigation strategy 2

## Related

<!-- Link related resources -->

- Closes #<issue_number>
- Related to #<issue_number>
- Depends on #<pr_number>
- Design doc: <link>
- Spike research: <link>

## Checklist

- [ ] Code follows project conventions
- [ ] All tests pass
- [ ] Documentation updated
- [ ] No linting errors
- [ ] Breaking changes documented
- [ ] Migration steps provided (if applicable)
- [ ] Security considerations addressed
```

### Step 8: Review and Validate

1. Ensure all sections are complete
2. Verify technical accuracy against actual changes
3. Check that links to issues are valid
4. Confirm testing strategy matches what was actually done
5. Validate that breaking changes are identified

## Output Expectations

### Success Criteria

- PR description is comprehensive and accurate
- All change categories are covered
- Technical details explain the "why" not just the "what"
- Testing strategy is clearly documented
- Breaking changes are identified with migration steps
- Related issues are linked
- Assumptions are explicitly stated
- Risk assessment is included

### Generated Output

Present the PR description in markdown format ready to paste into GitHub:

```
✅ Generated PR description for: <branch_name>

📊 Change Summary:
   - Files changed: <count>
   - Additions: <lines>
   - Deletions: <lines>
   - Categories: <list>

📝 Description ready to use:
   Copy the generated markdown and paste into GitHub PR description.

🔍 Review checklist:
   - [ ] All changes are documented
   - [ ] Testing strategy is complete
   - [ ] Breaking changes are identified
   - [ ] Related issues are linked
   - [ ] Migration steps provided (if needed)
```

## Validation Steps

1. **Review completeness**: Ensure all sections are filled
2. **Verify accuracy**: Check description matches actual changes
3. **Test links**: Confirm issue/PR links are valid
4. **Check formatting**: Ensure markdown renders correctly
5. **Validate against code**: Compare description to git diff

## Quality Assurance

- [ ] Summary is clear and concise (2-3 sentences)
- [ ] All changed files are documented
- [ ] Each change has a clear purpose explained
- [ ] Testing strategy matches actual testing performed
- [ ] Breaking changes are clearly identified
- [ ] Migration steps are actionable
- [ ] Related issues are linked
- [ ] Assumptions are explicit
- [ ] Risks are assessed with mitigation strategies
- [ ] Checklist is complete
- [ ] Markdown formatting is correct

## Edge Cases

- **No changes detected**: Report error and suggest checking branch/commits
- **Too many changes**: Summarize high-level categories, detail significant changes only
- **Mixed change types**: Clearly separate features, fixes, refactoring, etc.
- **No tests**: Warn that testing section is incomplete
- **Breaking changes not documented**: Flag as potential issue
- **Unclear commit messages**: Ask for clarification on change purpose
- **Multiple unrelated changes**: Suggest splitting into multiple PRs

## Related Resources

- [code-review-standards skill](/.github/skills/code-review-standards/SKILL.md)
- [Engineering guidelines](/.github/instructions/engineering.instructions.md)
- [GitHub PR Best Practices](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests)
- [Conventional Commits](https://www.conventionalcommits.org/)

## Example Usage

**Scenario**: Creating PR for ROS 2 package addition

```
Input: Branch name: feature/add-vision-package
Output: Comprehensive PR description including:
- Summary: Added drqp_vision package for image processing
- Features: Camera interface, image filters, object detection
- Tests: Unit tests for all algorithms, integration test with camera
- Dependencies: OpenCV 4.x, cv_bridge
- Breaking changes: None
- Related: Closes #145 (Add vision capability)
```
