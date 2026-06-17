# T3-A · Fill in `drqp_control` package description

**Severity:** Low (housekeeping)
**Agent turns:** 1
**File:** `packages/runtime/drqp_control/package.xml:6`

## Background

```xml
<description>TODO: Package description</description>
```

The ROS 2 package manifest has a placeholder description. This shows up in `ros2 pkg list`,
`rosdep`, and any tooling that reads the manifest.

## Acceptance criteria

- [ ] `<description>` contains a one-sentence summary of what the package provides.
- [ ] The TODO placeholder is gone.

## Suggested text

```xml
<description>ROS 2 control hardware interface and controllers for the Dr.QP hexapod robot.</description>
```

## Files to modify

| File | Change |
|------|--------|
| `packages/runtime/drqp_control/package.xml` | Replace placeholder with real description |

## Dependencies

None. Standalone one-line edit.
