---
name: example-good-skill
description: Toolkit for testing local web applications using Playwright browser automation. Use when asked to verify frontend functionality, debug UI behavior, capture browser screenshots, check for visual regressions, or view browser console logs. Supports Chrome, Firefox, and WebKit browsers for comprehensive cross-browser testing.
license: Complete terms in LICENSE.txt
---

# Example Good Skill

This skill demonstrates proper structure and best practices for Agent Skills.

## When to Use This Skill

Use this skill when you need to:

- Test web application functionality in a real browser
- Capture screenshots for visual regression testing
- Debug UI issues by inspecting browser console logs
- Verify cross-browser compatibility
- Automate user interactions for testing

## Prerequisites

Before using this skill, ensure you have:

- Node.js 18+ installed
- Playwright package installed (`npm install -D @playwright/test`)
- A local web server running (e.g., `npm run dev`)
- Access to the application under test

## Testing Workflow

### 1. Setup Test Environment

```bash
# Install Playwright browsers
npx playwright install
```

### 2. Run Tests

```bash
# Run all tests
npx playwright test

# Run specific test
npx playwright test example.spec.ts
```

## Debugging Workflow

### 1. Capture Screenshots

Use the screenshot tool to capture current page state:

```bash
npx playwright test --headed --debug
```

### 2. Inspect Console Logs

Review browser console for JavaScript errors and warnings.

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Browser not launching | Run `npx playwright install` to install browsers |
| Tests timing out | Increase timeout in playwright.config.ts |
| Flaky tests | Add explicit waits for elements |
| Screenshots differ | Update baseline screenshots |

## References

- [Playwright Documentation](https://playwright.dev)
- [Best Practices Guide](https://playwright.dev/docs/best-practices)
