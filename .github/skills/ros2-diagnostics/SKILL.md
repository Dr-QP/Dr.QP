---
name: ros2-diagnostics
description: Debug and troubleshoot ROS 2 systems using introspection tools. Use when asked to debug runtime issues, inspect topics and services, monitor message flow, analyze node communication, check system health, diagnose performance problems, analyze bag files, use rqt tools, or troubleshoot message frequency issues. Supports ros2 topic, ros2 service, ros2 node commands and rqt visualization tools.

---

# ROS 2 Diagnostics

Debug and troubleshoot ROS 2 systems using command-line introspection tools and graphical utilities for runtime analysis.

## When to Use This Skill

- Debug runtime communication issues between nodes
- Inspect topic message flow and frequencies
- Monitor service calls and responses
- Analyze node health and connectivity
- Check message types and content
- Diagnose performance bottlenecks
- Record and analyze bag files for debugging
- Visualize system architecture with rqt
- Troubleshoot "no messages received" issues
- Monitor CPU/memory usage of nodes

## Prerequisites

- ROS 2 Jazzy installation
- Running ROS 2 system to diagnose
- `ros2` command-line tools available
- Optional: `rqt` and `rqt_*` plugins for visualization
- Optional: `ros2 bag` for recording/playback
- Basic understanding of ROS 2 topics, services, nodes

## Diagnostic Tools Overview

| Tool | Purpose | Common Commands |
|------|---------|----------------|
| **ros2 topic** | Inspect topic communication | `list`, `echo`, `info`, `hz`, `bw` |
| **ros2 service** | Inspect service communication | `list`, `call`, `type` |
| **ros2 node** | Inspect node information | `list`, `info` |
| **ros2 interface** | Show message/service definitions | `show`, `list` |
| **ros2 bag** | Record and playback messages | `record`, `play`, `info` |
| **rqt_graph** | Visualize node/topic graph | GUI application |
| **rqt_console** | View node log messages | GUI application |

## Step-by-Step Workflows

### Workflow 1: Basic System Introspection

Get overview of running ROS 2 system.

1. List all running nodes:
   ```bash
   ros2 node list
   ```

2. List all active topics:
   ```bash
   ros2 topic list
   ```

3. List topics with message types:
   ```bash
   ros2 topic list -t
   ```

4. List all available services:
   ```bash
   ros2 service list
   ```

5. Check detailed information about a specific node:
   ```bash
   ros2 node info /<node_name>
   ```

   This shows:
   - Subscribers (topics node reads from)
   - Publishers (topics node publishes to)
   - Service servers (services node provides)
   - Service clients (services node uses)
   - Action servers and clients

**When to use**: Initial system overview, understanding node connectivity

### Workflow 2: Debug Topic Communication Issues

Investigate topics not receiving or sending messages correctly.

1. Check if topic exists:
   ```bash
   ros2 topic list | grep <topic_name>
   ```

2. Get detailed topic information:
   ```bash
   ros2 topic info /<topic_name> --verbose
   ```

   This shows:
   - Message type
   - Publisher count
   - Subscriber count
   - Quality of Service (QoS) settings

3. Monitor message publication rate:
   ```bash
   ros2 topic hz /<topic_name>
   ```

4. Check message bandwidth:
   ```bash
   ros2 topic bw /<topic_name>
   ```

5. View actual message content:
   ```bash
   ros2 topic echo /<topic_name>
   ```

6. View limited number of messages:
   ```bash
   ros2 topic echo /<topic_name> --once
   ros2 topic echo /<topic_name> --max-count 5
   ```

7. View specific message fields:
   ```bash
   ros2 topic echo /<topic_name> --field data
   ros2 topic echo /<topic_name> --field pose.position
   ```

8. Check message type definition:
   ```bash
   ros2 interface show <message_type>
   ```

**Common issues detected**:
- No publishers → upstream node not running
- No subscribers → downstream node not running
- Low hz → performance issue or computation bottleneck
- QoS mismatch → publishers and subscribers have incompatible QoS

**When to use**: "No messages received", communication problems, performance issues

### Workflow 3: Diagnose Service Communication

Debug service call issues and inspect service definitions.

1. List available services:
   ```bash
   ros2 service list
   ```

2. Check service type:
   ```bash
   ros2 service type /<service_name>
   ```

3. View service definition:
   ```bash
   ros2 interface show <service_type>
   ```

4. Call a service manually:
   ```bash
   ros2 service call /<service_name> <service_type> "<request_yaml>"
   ```

   Example:
   ```bash
   ros2 service call /set_parameters rcl_interfaces/srv/SetParameters \
     "{parameters: [{name: 'max_velocity', value: {type: 3, double_value: 2.0}}]}"
   ```

5. Find which node provides a service:
   ```bash
   ros2 node info /<node_name> | grep "Service Servers"
   ```

**When to use**: Service call failures, testing service interfaces

### Workflow 4: Record and Analyze Bag Files

Capture system data for offline analysis and debugging.

1. Record all topics:
   ```bash
   ros2 bag record -a -o <bag_name>
   ```

2. Record specific topics:
   ```bash
   ros2 bag record -o <bag_name> /<topic1> /<topic2> /<topic3>
   ```

3. Get bag file information:
   ```bash
   ros2 bag info <bag_name>
   ```

   Shows:
   - Duration
   - Start/end time
   - Message count per topic
   - Topic types

4. Play back recorded data:
   ```bash
   ros2 bag play <bag_name>
   ```

5. Play at different speed:
   ```bash
   ros2 bag play <bag_name> --rate 0.5  # Half speed
   ros2 bag play <bag_name> --rate 2.0  # Double speed
   ```

6. Play specific topics:
   ```bash
   ros2 bag play <bag_name> --topics /<topic1> /<topic2>
   ```

7. Loop playback:
   ```bash
   ros2 bag play <bag_name> --loop
   ```

**When to use**: Reproducing bugs, testing with recorded data, sharing debug info

### Workflow 5: Visualize System Architecture with rqt

Use graphical tools to understand system structure.

1. View node and topic graph:
   ```bash
   rqt_graph
   ```

   Features:
   - Shows nodes as ovals
   - Shows topics as rectangles
   - Shows connections between nodes
   - Filter by namespace or node
   - Refresh to see changes

2. View and filter log messages:
   ```bash
   rqt_console
   ```

   Features:
   - View DEBUG, INFO, WARN, ERROR, FATAL messages
   - Filter by node name
   - Filter by severity
   - Search message content
   - Export logs

3. Plot numeric data in real-time:
   ```bash
   rqt_plot
   ```

   Usage:
   - Add topic fields to plot (e.g., `/topic/data`)
   - Multiple topics on same plot
   - Auto-scaling or fixed scale
   - Save plot data

4. Monitor topic data in tree view:
   ```bash
   rqt_topic
   ```

5. Call services with GUI:
   ```bash
   rqt_service_caller
   ```

6. Launch multiple rqt plugins in one window:
   ```bash
   rqt
   ```

   Then: Plugins → Choose plugins to load

**When to use**: Visual debugging, understanding system architecture, presentations

### Workflow 6: Performance Monitoring and Analysis

Diagnose performance issues and resource usage.

1. Monitor node CPU and memory (requires system tools):
   ```bash
   top -p $(pgrep -f <node_executable>)
   ```

2. Check topic publication frequency over time:
   ```bash
   ros2 topic hz /<topic_name> --window 100
   ```

3. Monitor topic bandwidth:
   ```bash
   ros2 topic bw /<topic_name>
   ```

4. Check for dropped messages (requires bag recording):
   ```bash
   # Record with timestamps
   ros2 bag record /<topic_name> -o debug_bag

   # Analyze for gaps
   ros2 bag info debug_bag
   ```

5. Profile node startup time:
   ```bash
   time ros2 run <package> <executable>
   ```

6. Monitor parameter events:
   ```bash
   ros2 topic echo /parameter_events
   ```

7. Check basic ROS 2 graph and daemon health:
   ```bash
   ros2 daemon status  # Check ROS 2 daemon health
   ros2 node list      # Verify nodes are visible
   ```

**When to use**: Performance problems, high CPU usage, memory leaks

## Diagnostic Patterns

| Symptom | Likely Cause | Diagnostic Command |
|---------|--------------|-------------------|
| **Topic has no data** | No publisher running | `ros2 topic info /<topic> --verbose` |
| **Messages too slow** | Publisher rate issue | `ros2 topic hz /<topic>` |
| **High bandwidth** | Large messages or high rate | `ros2 topic bw /<topic>` |
| **Service fails** | Wrong request format | `ros2 interface show <srv_type>` |
| **Node not visible** | Node crashed or not started | `ros2 node list`, check logs |
| **QoS incompatibility** | Mismatched reliability/durability | `ros2 topic info /<topic> --verbose` |

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| "No topics found" | No nodes publishing | Check if nodes are running with `ros2 node list` |
| "Command not found: ros2" | Environment not sourced | Run `source scripts/setup.bash` |
| Topic echo shows nothing | Wrong topic name or no messages | Verify topic name with `ros2 topic list`, check publisher with `ros2 topic info` |
| rqt_graph empty | No active communication | Start nodes, ensure they're publishing/subscribing |
| Service call hangs | Service not available or node busy | Check service exists with `ros2 service list`, verify node is responsive |
| High hz but application slow | Processing bottleneck not in ROS | Profile application code, check callback processing time |
| Bag file won't play | Wrong format or corrupted | Check with `ros2 bag info`, verify SQLite3 format |
| rqt won't start | Missing Python dependencies | Install: `sudo apt install ros-jazzy-rqt-*` |

## Quality of Service (QoS) Debugging

Common QoS incompatibilities:

| Publisher QoS | Subscriber QoS | Result |
|---------------|----------------|--------|
| RELIABLE | BEST_EFFORT | ✅ Compatible |
| BEST_EFFORT | RELIABLE | ❌ Incompatible |
| TRANSIENT_LOCAL | VOLATILE | ✅ Compatible |
| VOLATILE | TRANSIENT_LOCAL | ❌ Incompatible |

Check QoS settings:
```bash
ros2 topic info /<topic> --verbose
```

## Common Diagnostic Workflows

### Debug "No Messages Received"
1. `ros2 topic list` → Verify topic exists
2. `ros2 topic info /<topic> --verbose` → Check publishers/subscribers
3. `ros2 topic echo /<topic>` → Verify messages are flowing
4. `ros2 topic hz /<topic>` → Check publication rate
5. Check QoS compatibility

### Debug Node Communication
1. `ros2 node list` → Verify nodes running
2. `ros2 node info /<node>` → Check connections
3. `rqt_graph` → Visualize connections
4. `ros2 topic echo` → Verify data flow

### Debug Performance Issues
1. `ros2 topic hz` → Check message rates
2. `ros2 topic bw` → Check bandwidth usage
3. `top` → Check CPU/memory usage
4. Profile code with appropriate tools

## References

- [ROS 2 CLI Tools](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html)
- [Understanding ROS 2 Topics](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [Using rqt_console](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html)
- [Recording and Playing Back Data](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
