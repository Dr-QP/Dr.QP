---
name: create-state-machine-python
description: Create a Python robotics finite-state machine with python-statemachine states, events, guards, callbacks, tests, and dependency declaration. Use when implementing behavior states or transitions in Python; use ros2-lifecycle-management for managed ROS lifecycle nodes.
---

# Create Python State Machine

Implement a domain state machine with `python-statemachine`; it is distinct
from a ROS 2 managed lifecycle node. For C++ state machines, use
[create-state-machine-cpp](../create-state-machine-cpp/). For ROS lifecycle
state transitions and lifecycle-node design, use
[ros2-lifecycle-management](../ros2-lifecycle-management/).

## Design the state model first

Require a machine name, at least two valid lower-snake-case states, an initial
state, events, permitted transitions, guard conditions, and effects. Produce a
small transition table before code. Reject an initial state outside the state
set and decide how callers observe an invalid event or failed guard (for
example, a typed error plus a ROS log at the node boundary).

Place reusable machine logic at
`<package_root>/<package_name>/<machine_name>_sm.py`; keep ROS publishers,
subscriptions, and timers in a separate node module. This keeps transitions
unit-testable without a running ROS graph.

## Implement with python-statemachine

Define `State` objects with one `initial=True` state and transitions named for
domain events. Keep entry/exit callbacks short; send I/O through an adapter or
node callback instead of embedding blocking ROS work in a transition. Provide
an explicit event-dispatch method only when callers need a stable domain API;
otherwise call the generated event method directly. Do not claim that these
domain states activate or deactivate a ROS lifecycle node.

Use the library name declared by the package:

```python
install_requires=['python-statemachine>=2.5.0']
```

Add it only when the package does not already declare the dependency. Do not
rewrite `setup.py` or add a console entry point when adding a non-executable
state-machine module. The library has no usable workspace rosdep key, so do not
invent a `package.xml` `python3-*` dependency; the package's Python install
requirement is the installation contract.

## Test and document behavior

Use [add-test-file-python](../add-test-file-python/) to add focused pytest
coverage for the initial state, every valid transition, rejected events,
guards, and callback effects. Add a Mermaid state diagram to the relevant
package README only when it helps explain a nontrivial public behavior model.

Build and test the owning package through the workspace wrapper:

```bash
scripts/with-ros-env.sh python3 -m colcon build \
  --packages-up-to <package_name> --symlink-install
scripts/with-ros-env.sh python3 -m colcon test \
  --packages-select <package_name> --mixin coverage-pytest \
  --return-code-on-test-failure
```

Use [ros2-workspace-testing](../ros2-workspace-testing/) for logs, reruns, and
the no-local-ROS escalation path.

## Related resources

- [python-statemachine documentation](https://python-statemachine.readthedocs.io/)
- [create-ros2-package-python](../create-ros2-package-python/)
- [ros2-lifecycle-management](../ros2-lifecycle-management/)
