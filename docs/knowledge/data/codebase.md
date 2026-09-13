---
type: hub
description: Codebase maps derived from the code, each pinned to the tracked-source fingerprint it was read from.
stage: living
generated:
  by: human:author
  at: 2026-08-01T00:00:00Z
---

# 🧭 Codebase

*The map of the code as it actually is — written only by reading the code, never
from memory. The map mirrors the code's containment tree: one doc per component
(crate, package, module) at a canonical key matching its source path, children
linked from their parent's `## Contains` — so `iwe tree -k data/codebase`
renders the component tree. Every doc carries `source` (the code it describes),
`source_digest` (a fingerprint of that code's tracked contents), and `verified`
(the date); a digest that no longer matches the code means the doc is suspect —
refresh it. Division of truth: spec/ is what must be, architecture/ is why it's
shaped this way, this hub is what is.*

## Getting around

*Filled when you map the codebase: how to build, run, and test; the entry
points; a directory → component table.*

✏️

## Components

[Timer engine](codebase/timer.example)

[Session store](codebase/store.example)

## Flows

[Flow: session lifecycle](codebase/flow-session-lifecycle.example)

## Interfaces

*External surfaces — HTTP APIs, CLI commands, storage formats, IPC contracts —
one doc each, keyed `api-<name>`.*
