# T3-D · Update IK notebook diagram to match Dr.QP leg geometry

**Severity:** Low (documentation)
**Agent turns:** 2
**File:** `docs/source/notebooks/1_getting_started_with_robot_ik.md:291`

## Background

```
TODO: Update diagram with the one from Dr.QP leg and names of angles and planes
      matching this notebook
```

The getting-started IK notebook has a placeholder referencing a diagram that should show the
Dr.QP leg geometry with labelled angles (coxa, femur, tibia) and the coordinate planes used
in the notebook's derivations. The current diagram is either absent or shows a generic robot
leg that does not match Dr.QP's physical dimensions and angle conventions.

## Goal

Replace or add a diagram that shows:

- The Dr.QP leg link layout (coxa / femur / tibia).
- The angle names as used in the notebook (`theta_coxa`, `theta_femur`, `theta_tibia` or
  equivalent).
- The relevant planes (horizontal plane for coxa rotation, sagittal plane for femur/tibia).
- Approximate link lengths from the URDF or the notebook constants.

## Acceptance criteria

- [ ] The diagram is present and referenced correctly in the notebook.
- [ ] Angle and plane labels match the variable names used in the notebook's IK equations.
- [ ] The TODO comment is removed from the notebook source.

## Implementation approach

### Turn 1 — Extract geometry from URDF and notebook

Read the URDF (or xacro) to get link lengths and joint axis definitions. Read the notebook to
identify which angle/plane names are used in the derivations. Produce a schematic (SVG, PNG,
or an embedded matplotlib diagram generated in a notebook cell).

### Turn 2 — Embed and verify

Insert the image reference or the generative notebook cell at line 291. Build the docs
(`sphinx-build` or `jupyter nbconvert`) and confirm the diagram renders correctly.

## Files to modify

| File                                                       | Change                                     |
| ---------------------------------------------------------- | ------------------------------------------ |
| `docs/source/notebooks/1_getting_started_with_robot_ik.md` | Remove TODO; add diagram reference or cell |
| `docs/source/notebooks/images/` (or equivalent)            | Add diagram file (if static image)         |

## Dependencies

- Best done alongside any other notebook refresh to avoid touching the file twice.
- No dependency on other TODO items.
