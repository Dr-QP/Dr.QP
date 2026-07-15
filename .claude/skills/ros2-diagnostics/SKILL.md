---
name: ros2-diagnostics
description: Diagnose a running ROS 2 system with graph, topic, service, action, QoS, bag, and process introspection. Use when messages, services, actions, nodes, performance, or recorded data are failing or behaving unexpectedly. Use ros2-launch-management to change a launch setup and ros2-parameter-tuning to change configuration.
---

# ROS 2 Diagnostics

Inspect first, then make the smallest safe reproduction. This skill is for a
running system; it does not change launch files or tune parameters.

## Run Commands Reliably

Run every ROS command through the workspace wrapper so it uses the overlay:

```bash
scripts/with-ros-env.sh ros2 <verb> <arguments>
```

In Codex, request sandbox escalation for commands that need ROS discovery
sockets, start processes, or write ROS logs. Explain that the command needs the
ROS runtime and its graph sockets. Do not treat a missing local ROS install as a
diagnostic result: follow the workspace container/Codespaces escalation route.

Use `./.tmp/` for disposable bags, captures, and command output. Create it
before recording; do not put diagnostic artifacts beside source files.

```bash
mkdir -p ./.tmp
```

## Triage the Graph

Start with the narrowest question and keep the output that supports a finding.

```bash
scripts/with-ros-env.sh ros2 node list
scripts/with-ros-env.sh ros2 topic list -t
scripts/with-ros-env.sh ros2 service list -t
scripts/with-ros-env.sh ros2 action list -t
scripts/with-ros-env.sh ros2 node info /<node_name>
```

`ros2 node info` is the authoritative per-node view: use it to compare the
expected publishers, subscribers, service servers/clients, and action
servers/clients with the graph. If nothing appears, check that the target
process is in the intended ROS domain before changing QoS or code.

## Diagnose Topic Delivery

```bash
scripts/with-ros-env.sh ros2 topic info /<topic_name> --verbose
scripts/with-ros-env.sh ros2 topic echo /<topic_name> --once --timeout 5
scripts/with-ros-env.sh ros2 topic echo /<topic_name> --field data --once --timeout 5
scripts/with-ros-env.sh ros2 topic hz /<topic_name> --window 100
scripts/with-ros-env.sh ros2 topic bw /<topic_name>
```

`ros2 topic echo` in Jazzy has no `--max-count` option. Use `--once` with a
timeout for a bounded probe; use an external, explicitly time-bounded command
only when several samples are genuinely required. Check the message definition
before interpreting a field:

```bash
scripts/with-ros-env.sh ros2 topic type /<topic_name>
scripts/with-ros-env.sh ros2 interface show <package>/msg/<Message>
```

Interpret the evidence in order: no publisher means an upstream startup or
namespace issue; a publisher but no compatible subscription points to QoS;
normal rate with a slow application points to callback work rather than graph
delivery. `topic info --verbose` exposes endpoint QoS for that comparison.

## Diagnose Services and Actions

Inspect a service without calling it:

```bash
scripts/with-ros-env.sh ros2 service type /<service_name>
scripts/with-ros-env.sh ros2 interface show <package>/srv/<Service>
```

ROS 2 has no CLI command that maps one service name directly to its provider.
After confirming the service name, inspect candidate nodes and find the node
whose `Service Servers:` section lists that exact name:

```bash
scripts/with-ros-env.sh ros2 node list
scripts/with-ros-env.sh ros2 node info /<candidate_node>
```

Call a service only with permission to exercise the interface; it can alter the
running system.

```bash
scripts/with-ros-env.sh ros2 service call /<service_name> <package>/srv/<Service> \
  "<request_yaml>"
```

Actions need their own path; a topic-only inspection can miss a failed action
server or goal lifecycle:

```bash
scripts/with-ros-env.sh ros2 action list -t
scripts/with-ros-env.sh ros2 action type /<action_name>
scripts/with-ros-env.sh ros2 action info /<action_name>
```

`send_goal` is an operation, not an inspection. Use it only with a harmless
goal and authorization to trigger the action.

## Capture a Reproducible Sample

Record only relevant topics and write to `./.tmp/`:

```bash
mkdir -p ./.tmp
scripts/with-ros-env.sh ros2 bag record -o ./.tmp/diagnostic-bag \
  /<topic_one> /<topic_two>
scripts/with-ros-env.sh ros2 bag info ./.tmp/diagnostic-bag
scripts/with-ros-env.sh ros2 bag play ./.tmp/diagnostic-bag --rate 0.5
```

Stop recording cleanly with Ctrl-C before inspecting or replaying the bag. Do
not record all topics by default; it can conceal the signal and consume storage.

For a visual graph or logs, launch the applicable GUI through the same wrapper:

```bash
scripts/with-ros-env.sh rqt_graph
scripts/with-ros-env.sh rqt_console
```

## Report and Route Findings

Record the command, namespace/domain, observed endpoints/QoS, and the smallest
evidence that supports the conclusion. Route the next change deliberately:

- Use `ros2-parameter-tuning` for a configuration or runtime parameter change.
- Use `ros2-launch-management` for launch arguments, composition, or startup
  ordering.
- Use `ros2-workspace-build` or `ros2-workspace-testing` when a source change
  needs a focused build or test.

## References

- [ROS 2 CLI tools](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html)
- [Topics](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [Services](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [Actions](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- [Bags](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
