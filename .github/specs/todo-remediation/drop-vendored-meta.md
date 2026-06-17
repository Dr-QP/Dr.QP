# DROP · Vendored and meta TODO annotations — no action required

These 29 items appear in code or files that the project does not own and should not modify.
They are documented here so they can be excluded from future audits.

## Vendored ROS 2 launch library (11 items)

**Path prefix:** `packages/vendor/launch/launch/launch/`

All annotated by upstream contributors (wjwwood, ivanpauno, hidmic, dhood, jacobperron,
jacobperron). These are tracked in the upstream `ros2/launch` repository.

| File | Line | Author | Note |
|------|------|--------|------|
| `substitutions/python_expression.py` | 120 | — | Backward compat note |
| `launch_service.py` | 216 | wjwwood | SIGQUIT subprocess cleanup |
| `logging/handlers.py` | 53 | hidmic | Module `__getattr__` migration |
| `logging/__init__.py` | 83 | hidmic | TOCTTOU race in log dir creation |
| `event_handlers/on_shutdown.py` | 54 | wjwwood | Callable signature validation |
| `event_handlers/on_shutdown.py` | 69 | dhood | Known-actions description |
| `event_handlers/on_process_io.py` | 33 | wjwwood | `__init__` flexibility |
| `event_handlers/on_action_event_base.py` | 89 | wjwwood | Callable signature validation |
| `event_handlers/on_action_event_base.py` | 127 | jacobperron | Entity description |
| `frontend/expose.py` | 71 | ivanpauno | Signature check |
| `frontend/expose.py` | 72 | ivanpauno | Annotation inference |
| `actions/execute_local.py` | 343 | wjwwood | Windows SIGINT workaround |
| `actions/execute_local.py` | 752 | — | `OnProcessExit` None callable |
| `actions/log_info.py` | 21 | — | Remove after L-turtle release |
| `actions/log.py` | 62 | — | Remove after Python 3.11 min |
| `actions/log.py` | 95 | — | Remove after L-turtle release |
| `launch_introspector.py` | 51 | wjwwood | Text formatting edge case |
| `launch_introspector.py` | 80 | wjwwood | Complex branching description |
| `substitutions/python_expression.py` | 84 | — | XXX type annotation confusion |

## Vendored rapidjson library (5 items)

**Path prefix:** `packages/runtime/drqp_rapidjson/include/drqp_rapidjson/`

| File | Line | Note |
|------|------|------|
| `schema.h` | 2204 | Return type question |
| `schema.h` | 2278 | Cache pointer↔id mapping |
| `schema.h` | 2300 | Cache pointer↔id mapping (duplicate) |
| `schema.h` | 2323 | Cache pointer↔id mapping (FindId) |
| `reader.h` | 1722 | StrtodX overflow reporting |

## Ansible configuration template (3 items)

**File:** `docker/ros/ansible/ansible.cfg`

Lines 343, 693, 696 carry `# TODO: write it` placeholders that are part of the Ansible
upstream template. The project does not maintain this file's content beyond configuration
values it actively sets.

## Documentation examples (2 items)

**File:** `.github/instructions/agent-skills.instructions.md` lines 300 and 303

The word "TODO" appears inside a Markdown code block used as an example of how to structure
agent workflow checklists. It is not a project annotation.

**File:** `.github/skills/implement-publisher-subscriber/SKILL.md` line 46

"Include copyright, TODO for message population." is instruction text telling a code
generator to emit a TODO comment in generated code. Not a project annotation.

## Recommended suppression

Add the following paths to any lint/grep-based TODO scanner to prevent these from resurfacing:

```
packages/vendor/
packages/runtime/drqp_rapidjson/
docker/ros/ansible/ansible.cfg
.github/instructions/
.github/skills/
```
