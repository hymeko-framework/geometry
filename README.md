# geometry

Surfaces and the processing done on them, for every HyMeKo consumer that handles meshes: the
renderer, the simulators, the XR hosts and the asset tools.

## What belongs here

Geometry processing on its own terms: indexed meshes and spline surfaces, their validation and
topology, normals, fairing, regularisation, subdivision, solid operations, simplification into
levels of detail, interchange-format import, and the surface materials that dress them.

## What does not

- Reading HyMeKo IR. Links, joints, shapes declared in a description and the poses they imply are
  `hymeko_geometry` in the transformation repository, which builds on this one.
- Numerical algebra with no mesh concept: the tensor repository (rotors, splines as functions,
  solvers).
- Drawing: the renderer (Kawamirai) owns GPU upload, culling, picking and shading.
- Hypergraph algorithms over IR structure: hotaru.

## Crates

| crate | contents | dependencies |
|---|---|---|
| `hymeko_mesh` | `mesh`: `TriangleMesh`, topology, normals, fairing, regularisation, subdivision, solids, spline surfaces, LODs; `material`: PBR materials, radiance images, the procedural material library | serde, serde_json; meshopt with `mesh-optimization` |

`mesh-optimization` builds meshoptimizer (C++) through `cc`. Leave it off where no C++ toolchain
for the target exists.

## Checks

```
cargo test --offline -j 2
cargo test --offline -j 2 --features mesh-optimization
```

## Provenance

`hymeko_mesh` is `hymeko_geometry::{mesh, material}` moved out of the transformation repository at
c90eb2b5dca72693371df99d899e3b04c3dd32f9; their history is there. `hymeko_geometry` re-exports them
so existing paths keep compiling while consumers move to this crate.
