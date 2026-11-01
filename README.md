# geometry

Surfaces, kinematic models and the geometric computations on them, for every HyMeKo consumer
that handles meshes or mechanisms: the renderer, the simulators, the XR hosts and the asset tools.
Nothing here reads HyMeKo IR.

## What belongs here

- Geometry processing on its own terms: indexed meshes and spline surfaces, their validation and
  topology, normals, fairing, regularisation, subdivision, solid operations, simplification into
  levels of detail, interchange-format import, and the surface materials that dress them.
- Kinematic models as plain data (links, joints, frames, actuation) and the computations on them:
  forward and inverse kinematics, Denavit–Hartenberg chains, joint playout, analytic two-bone
  limbs, and collision geometry (shapes, placement, broad and narrow phase).

## What does not

- Reading HyMeKo IR. Extracting links, joints, shapes and actuation from a compiled description
  is `hymeko_geometry` in the transformation repository, which builds on this one and re-exports
  it with Core declaration handles.
- Physics stepping: articulated-body dynamics, contact solving, the engine, muscles, force fields
  and vehicles are the kawakern repository, which builds on `hymeko_kinematics`.
- Numerical algebra with no geometry concept: the tensor repository (rotors, splines as
  functions, solvers).
- Drawing: the renderer (Kawamirai) owns GPU upload, culling, picking and shading.
- Hypergraph algorithms over IR structure: hotaru.

## Crates

| crate | contents | dependencies |
|---|---|---|
| `hymeko_mesh` | `mesh`: `TriangleMesh`, topology, normals, fairing, regularisation, subdivision, solids, spline surfaces, LODs; `material`: PBR materials, radiance images, the procedural material library; `recipe`: validated processing recipes and their execution | serde, serde_json; meshopt with `mesh-optimization` |
| `hymeko_kinematics` | `model`, `joints`, `actuation`: the kinematic model; `forward`, `ik`, `dh`, `playout`, `two_bone`, `collide` | with `model` (default): tensor's clifford, tensor, spline; graph's sbsh. `two_bone` needs none |

`mesh-optimization` builds meshoptimizer (C++) through `cc`. Leave it off where no C++ toolchain
for the target exists.

### Handles

A kinematic model keeps its source's identifiers on links, joints, attachments and drives, so a
consumer maps a part back to what declared it without a name lookup. The types take the handle
family as a parameter, `KinematicModel<H: Handles>`; geometry only compares and hashes the ids and
treats `H::NONE` as "no source declaration". `Indexed` (plain `usize`) serves hand-built models.
Transformation's `hymeko_geometry` reads models with Core `DeclId`s and re-exports the types as
aliases, so its consumers see no parameter.

## Checks

```
cargo test --offline -j 2
cargo test --offline -j 2 --features hymeko_mesh/mesh-optimization
cargo check --offline -j 2 -p hymeko_kinematics --no-default-features
```

## Provenance

`hymeko_mesh` is `hymeko_geometry::{mesh, material}` moved out of the transformation repository at
c90eb2b5dca72693371df99d899e3b04c3dd32f9; their history is there.

`hymeko_kinematics` is the IR-free half of `hymeko_geometry::{kinematics, forward, two_bone,
collide}` moved out of the transformation repository at
232f720e1824bf4c2822cce74906118743a008a9, with the handle parameter added and `FkPlan`'s topology
exposed through `parents()`/`order()` for kawakern; their history is there.

`hymeko_mesh::recipe` is `hymeko_geometry::pipeline`'s IR-free recipe runner, moved on 2026-11-01;
projecting a recipe out of IR stays in transformation (`hymeko_mechanism::recipe`).

`hymeko_geometry` re-exports both crates so existing paths keep compiling while consumers move.
