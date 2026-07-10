---
id: RM-11
title: Autonomous docking and self-charging
status: proposed
depends_on: [RM-01, RM-05, RM-06] # RM-01 battery telemetry, RM-05 dock pose + tag detector, RM-06 nav
packages: [drqp_robot_mcp, drqp_interfaces, drqp_navigation, 'NEW: drqp_docking']
---

# RM-11 — Docking and charging

## Objective

Close the autonomy loop: the robot returns to a charging dock, aligns precisely enough to make
electrical contact, confirms charging, and manages its own energy budget — so it can operate
unattended, which is what "autonomous home pet" ultimately requires. Navigation gets it near the
dock (RM-06); a dedicated visual-servo approach handles the final centimetres where map accuracy
(RM-05's ~0.15 m) is far too coarse for contact.

## Interfaces

- **Actions** (NEW node `docking_server`, in `drqp_docking`): `robot/dock` and `robot/undock`
  (`drqp_interfaces` actions with feedback: `staging | approaching | aligning | engaged | charging`
  / `undocking | clear`). `robot/dock` navigates to a staging pose near the dock via Nav2
  `NavigateToPose`, then runs the dock-approach controller for final alignment; `robot/undock`
  backs out to a safe clearance pose before returning control.
- **Dock pose**: `dock` entry in RM-05 `locations.yaml` (staging pose in `map`) plus an AprilTag on
  the dock (reuse the RM-05 `apriltag` detector) giving the precise relative pose for the final
  approach. Tag id/size in `dock.yaml`.
- **Dock-approach controller**: adopt Nav2 `opennav_docking` if it fits, else a thin visual-servo
  controller emitting `cmd_vel` (through the RM-02 path/mux) to null the tag-relative pose error.
- **Charge confirmation**: read charging voltage/current rising on the RM-01 battery interface to
  confirm engagement (`engaged` → `charging`); retry approach on no-contact; report `charge_full`
  on plateau.
- **Energy policy** (small supervisor, deterministic): dock when `/battery_state` below a
  threshold **with hysteresis**, or on idle timeout; undock on command, `charge_full`, or a
  scheduled activity. Emits intent to RM-07's behavior engine rather than commanding motion
  directly.
- **MCP**: `robot.dock`, `robot.undock`, `robot.battery_status` in `drqp_robot_mcp/server.py`.
- **Behavior integration** (RM-07 Track C): the low-battery/rest behavior triggers `robot/dock`;
  greeting/interaction preempts docking per priority.

## Sim assets

Dock model (AprilTag-marked, with an alignment funnel geometry) placed in the RM-05 `apartment.sdf`
world at the `locations.yaml` `dock` pose. Simulated charging: battery model voltage rises while
the robot holds the engaged pose within tolerance, falls otherwise.

## Acceptance criteria

- [ ] Launch test: from across the apartment, `robot/dock` reaches staging via Nav2, then aligns
      by tag to < 2 cm / 5° of the engaged pose and reports `engaged`.
- [ ] Charge-confirm test: holding the engaged pose ⇒ simulated battery rises and `charging` is
      reported; leaving it ⇒ `charging` drops.
- [ ] Low-battery autonomy test: injected low `/battery_state` ⇒ behavior engine triggers
      `robot/dock` with no external command; robot docks.
- [ ] Failed-approach retry test: perturb the approach (offset/occluded tag) ⇒ controller retries
      a bounded number of times, then aborts cleanly — never repeatedly drives into the dock.
- [ ] Undock test: `robot/undock` backs out to the clearance pose and returns the robot to normal
      `torque_on` operation.
- [ ] Hysteresis test: battery hovering at the threshold does not thrash dock/undock.
- [ ] Hardware: ≥ N docking attempts success-rate recorded; charge engagement + current verified;
      contact/thermal sanity documented.

## Constraints

- Docking motion uses the RM-02 `cmd_vel` path + `twist_mux`: **joystick preempts, kill switch
  stops**, and docking never bypasses the state machine.
- Charging is fail-safe: **no stride/torque while charging** (torque-off or hold at the dock);
  undock requires an explicit transition out of the charging state.
- The final contact phase (visual servo) is safety-local (runs on-robot); Nav2 may be off-board and
  is only used up to the staging pose.
- No custom charging electronics logic in ROS beyond reading the existing battery interface;
  contact/charger hardware design documented in hardware docs, not code.

## Follow-up (separate spec when picked up)

Battery health/runtime estimation (state-of-charge model, cycle logging) feeding the energy policy
and the RM-07 agent's self-reporting ("I need to charge soon").
