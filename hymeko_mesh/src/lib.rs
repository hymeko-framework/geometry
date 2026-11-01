//! # `hymeko_mesh` — surfaces and their appearance, without the IR
//!
//! Indexed triangle meshes and what is done to them, and the materials that dress them:
//!
//! - [`mesh`] — [`mesh::TriangleMesh`] and its validation and bounds; adjacency and manifold
//!   checks, seam-aware normals, fairing, constrained quad regularisation, Catmull–Clark
//!   subdivision, solid union, spline control surfaces, and, with `mesh-optimization`,
//!   meshoptimizer levels of detail.
//!   The [`mesh::closure`] query classifies any triangle list as closed or names its defect, and
//!   [`mesh::primitive`] builds the unit cube, quad, UV sphere and cylinder.
//! - [`ray`] — where a line meets a unit solid under an affine map, a capsule, or a triangle list.
//! - [`material`] — glTF metallic-roughness materials, radiance images and the shared procedural
//!   material library.
//!
//! Nothing here reads HyMeKo IR or draws anything. Reading the IR geometrically — links, joints,
//! the poses they imply — is `hymeko_geometry`'s, in the transformation repository, which builds on
//! this crate; drawing is the renderer's. Moved from `hymeko_geometry` (transformation
//! c90eb2b5dca72693371df99d899e3b04c3dd32f9), whose history these modules keep.

#![allow(missing_docs)] // inherited from the moved modules; tightened per-module as they are revisited

pub mod material;
pub mod mesh;
pub mod ray;
pub mod recipe;
