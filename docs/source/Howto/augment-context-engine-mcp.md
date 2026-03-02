# Augment Context Engine MCP

This page documents how to configure the Augment Context Engine MCP server for GitHub Copilot in this workspace.

## Install Auggie CLI

```
npm install -g @augmentcode/auggie@latest
```

## Sign in

```
auggie login
```

This opens a browser window for authentication.

## Configure MCP server

Create [/.vscode/mcp.json](.vscode/mcp.json) at the repository root with the following content:

```json
{
  "servers": {
    "augmentcode": {
      "type": "stdio",
      "command": "auggie",
      "args": ["--mcp", "--mcp-auto-workspace"]
    }
  },
  "inputs": []
}
```

## Test the integration

In GitHub Copilot Agent Mode, prompt:

```
What is this project? Please use codebase retrieval tool to get the answer.
```

Copilot should confirm it has access to the codebase-retrieval tool.
