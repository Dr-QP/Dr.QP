---
type: codebase
description: Small numpy-backed geometry primitives — 2D/3D points, lines, a leg polyline, and 4×4 affine transforms — shared by the kinematics models, brain and notebooks.
source: packages/runtime/drqp_kinematics/drqp_kinematics/geometry
source_digest: sha256:29137a726fc51753bb586d2e264f6d488d39777bdaf12e5163137eff3a026e0a
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: code
  resource: packages/runtime/drqp_kinematics/drqp_kinematics/geometry
---

# geometry

The value types every other Python package builds on. The subpackage re-exports:

- `Point` and `SimplePoint3D`
- `Point3D`
- `Line` and `Line3D`
- `Leg3D`
- `AffineTransform`

## Public surface

- **`Point3D(array)`:** wraps a numpy array. It has `x`/`y`/`z` accessors,
  arithmetic, `interpolate(other, alpha)`, `normalized()`, `copy()`, and
  `to_vector3()`, which returns a `geometry_msgs/Vector3`. It also has
  `xy`/`xz`/`yz` projections for plotting.
- **`AffineTransform(matrix)`:**
  - Constructors: `identity()`, `from_rotvec(…, degrees=False)`,
    `from_rotmatrix`, `from_translation`, `make(rotation, translation)`.
  - Accessors: `rotation`, `translation`, `matrix`, `inverse()`.
  - Application: `apply_point`, `apply_line`, `apply_nd`, and composition with
    `@`.
- **`Line`, `Line3D`, `Leg3D`:** labelled segments used for drawing legs in the
  notebooks.

## How it works

`AffineTransform` stores a 4×4 homogeneous matrix. Rotations go through
`scipy.spatial.transform.Rotation`. Composition is matrix multiplication, so
`A @ B` applies `B` first.

## Depends on

- `numpy`, `scipy`, and `geometry_msgs` (only for `to_vector3`).

## Invariants & gotchas

- `Point3D` is mutable: its setters write into the array. Code that caches
  targets (for example `WalkController.leg_tips_on_ground`) copies them with
  `.copy()` first.
- The 2D `Point` and `SimplePoint3D` types remain for notebook plotting. Runtime
  code uses `Point3D`.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_kinematics/drqp_kinematics/geometry/__init__.py` —
  re-exports
- `packages/runtime/drqp_kinematics/drqp_kinematics/geometry/point.py:104` —
  `Point3D`
- `packages/runtime/drqp_kinematics/drqp_kinematics/geometry/point.py:162` —
  `interpolate`
- `packages/runtime/drqp_kinematics/drqp_kinematics/geometry/point.py:197` —
  `to_vector3`
- `packages/runtime/drqp_kinematics/drqp_kinematics/geometry/transforms.py:28` —
  `AffineTransform`
- `packages/runtime/drqp_kinematics/drqp_kinematics/geometry/transforms.py:102`
  — `__matmul__`
