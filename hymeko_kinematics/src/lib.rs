//! Kinematic models and the computations on them, without IR: forward and inverse kinematics,
//! Denavit–Hartenberg chains, joint playout, analytic two-bone limbs and collision geometry.
//!
//! Moved out of the transformation repository's `hymeko_geometry`, which keeps reading these
//! models out of a HyMeKo IR and re-exports this crate with Core declaration handles. Physics
//! stepping over the same model is the kawakern repository.

/// Analytic two-bone limb IK, dependency-free (no `model` feature needed).
pub mod two_bone;

#[cfg(feature = "model")]
pub mod actuation;
#[cfg(feature = "model")]
pub mod collide;
#[cfg(feature = "model")]
pub mod dh;
#[cfg(feature = "model")]
pub mod forward;
#[cfg(feature = "model")]
pub mod handles;
#[cfg(feature = "model")]
pub mod ik;
#[cfg(feature = "model")]
pub mod joints;
#[cfg(feature = "model")]
pub mod model;
#[cfg(feature = "model")]
pub mod playout;

#[cfg(feature = "model")]
pub use handles::{Handles, Indexed};
