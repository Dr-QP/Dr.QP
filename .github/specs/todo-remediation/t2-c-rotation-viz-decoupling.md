# T2-C · Remove rotation-gait branch from gait visualiser

**Severity:** Medium
**Agent turns:** 5
**File:** `docs/source/notebooks/plotting/gaits.py:230`

## Background

The gait plotter contains domain logic that belongs in the gait generator:

```python
if _rotation_gaits:
    # TODO(anton-matosov): Why does visualization code need to know about rotations?
    rotation_transform = AffineTransform.from_rotvec(
        [0, 0, offset.x], degrees=True
    )
    leg_tip = rotation_transform.apply_point(leg_tip) + Point3D([0, 0, offset.z])
else:
    leg_tip = leg_tip + offset
```

`get_offsets_at_phase_for_leg()` returns semantically different data depending on gait type:

- For translation gaits: a 3D Cartesian displacement (`offset.x` is a length).
- For rotation gaits: `offset.x` is an **angle in degrees**, not a length.

The plotter compensates by detecting the gait type and applying a rotation transform itself.
This is a layering violation: the visualiser should receive ready-to-use world-frame points
regardless of gait type.

## Goal

Make `get_offsets_at_phase_for_leg()` (or a new method) return a consistent world-frame
`Point3D` leg-tip position for all gait types, so the plotter loop becomes unconditional.

## Acceptance criteria

- [ ] The `if _rotation_gaits:` branch is removed from `gaits.py`.
- [ ] All existing gait plots (wave, tripod, ripple, rotation variants) produce visually
      identical output to the current implementation.
- [ ] The gait generator module (`drqp_brain`) has corresponding unit tests that cover the
      rotation-gait output format change.
- [ ] No change to the public API visible to callers outside the plotter.

## Implementation approach

### Turn 1 — Trace the data path

Identify where `get_offsets_at_phase_for_leg` is defined and all its callers. Understand the
full contract: what does `offset.x` mean for a rotation gait today? What is `_leg_centers`
used for? Identify whether changing the return value breaks any other caller.

### Turn 2 — Design the fix

Two options:

**Option A — Fix in the generator:** Add a `get_world_tip_at_phase_for_leg(leg, phase, leg_center, …)`
method that internalises the rotation transform. The plotter calls this instead of
`get_offsets_at_phase_for_leg`.

**Option B — Fix the offset semantics:** Change `get_offsets_at_phase_for_leg` for rotation
gaits to return a Cartesian displacement computed from the angular offset + leg centre, making
the return type uniform. Update existing tests.

Option A is lower risk (additive); Option B is a cleaner API but changes existing behaviour.

### Turn 3 — Implement

Implement the chosen option. Keep `get_offsets_at_phase_for_leg` signature backward compatible
or update all callers if changing semantics.

### Turn 4 — Update the plotter

Replace the conditional branch with the unconditional new call. Confirm plots are visually
identical by running the notebook or the plot helper and diffing or visually comparing output.

### Turn 5 — Update/add tests

Add or update unit tests in `drqp_brain` that cover the rotation-gait output. Ensure the
generator tests cover both translation and rotation cases.

## Files to modify

| File                                                              | Change                              |
| ----------------------------------------------------------------- | ----------------------------------- |
| `docs/source/notebooks/plotting/gaits.py`                         | Remove `if _rotation_gaits:` branch |
| `packages/runtime/drqp_brain/…/gait_generator.py` (or equivalent) | Add/fix method                      |
| `packages/runtime/drqp_brain/test/…`                              | Add/update tests for gait generator |

## Dependencies

- Requires understanding of the gait generator internals before starting.
- No dependency on other TODO items.
