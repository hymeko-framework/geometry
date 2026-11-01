//! The kinematic model: links, joints, frames and actuation, as plain data.
//!
//! URDF, SDF, Gazebo and MJCF emission, forward and inverse kinematics, collision and the
//! physics engine all consume this one shape. Reading it out of a HyMeKo IR is the
//! transformation repository's `hymeko_geometry::kinematics`; nothing here reads IR.

use crate::handles::Handles;
use crate::joints::JointInfo;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeometryShape {
    Box,
    Cylinder,
    Sphere,
    /// A capsule (a cylinder capped by two hemispheres). Dimensional contract (2026-07-23, Galambos planar
    /// extension): `dimensions[0]` = the **rod/segment length** (the cylinder-axis extent, i.e. the `fromto`
    /// endpoint distance used by the golden MuJoCo builder — NOT including the hemispherical caps, which the
    /// emitter adds as `size` beyond each endpoint); `dimensions[1]` = the capsule **radius**.
    Capsule,
}

/// Explicit collision categories for a geometry (additive, 2026-07-23). Maps directly to MuJoCo
/// `contype` / `conaffinity`. `None` (no `collision {}` sub-node) preserves the historical behaviour exactly
/// (the emitter omits both attributes → MuJoCo defaults).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CollisionMask {
    pub contype: i64,
    pub conaffinity: i64,
}

#[derive(Debug, Clone)]
pub struct GeometryInfo {
    pub shape: GeometryShape,
    pub dimensions: Vec<f64>,
    /// Optional explicit collision mask; `None` ⇒ historical default behaviour (emitter omits contype/conaffinity).
    pub collision: Option<CollisionMask>,
}

#[derive(Debug, Clone)]
pub struct LinkInfo<H: Handles> {
    pub did: H::Id,
    pub name: String,
    pub mass: Option<f64>,
    pub geometry: Option<GeometryInfo>,
    pub origin: Option<Vec<f64>>,
    pub color: Option<Vec<f64>>,
    /// COM inertia in the link axes, kg m^2: xx, yy, zz, xy, xz, yz.
    /// Absent retains the historical point-mass numerical model.
    pub inertia_com: Option<[f64; 6]>,
}

#[derive(Debug, Clone)]
pub struct KinematicModel<H: Handles> {
    pub name: String,
    pub links: Vec<LinkInfo<H>>,
    pub joints: Vec<JointInfo<H>>,
    /// Declared `frame` elements, by name.
    ///
    /// A frame is not a body: it carries no mass and no geometry, and nothing about it is posed. It
    /// exists to be joined TO — `world` is the frame a floating base hangs from — so a joint whose
    /// parent is a frame anchors its child to something outside the mechanism rather than to
    /// another part of it. Kinematics has to be able to tell that apart from a joint that names a
    /// link which does not exist, and the only place that distinction is available is here.
    pub frames: Vec<String>,
    /// Geometric force transmissions over the existing links and their handles.
    pub actuation: crate::actuation::Actuation<H>,
}

impl<H: Handles> LinkInfo<H> {
    /// A physical inertia has positive principal moments and satisfies their triangle inequalities.
    pub fn valid_inertia(&self) -> bool {
        let Some([xx, yy, zz, xy, xz, yz]) = self.inertia_com else {
            return true;
        };
        if ![xx, yy, zz, xy, xz, yz].iter().all(|x| x.is_finite())
            || self.mass.is_none_or(|m| !m.is_finite() || m <= 0.0)
        {
            return false;
        }
        let determinant =
            xx * yy * zz + 2.0 * xy * xz * yz - xx * yz * yz - yy * xz * xz - zz * xy * xy;
        if xx <= 0.0 || xx * yy <= xy * xy || determinant <= 0.0 {
            return false;
        }
        // S = trace(I)/2 E - I is the second moment matrix; it must be positive semidefinite.
        let [a, b, c] = [
            (yy + zz - xx) / 2.0,
            (xx + zz - yy) / 2.0,
            (xx + yy - zz) / 2.0,
        ];
        let eps = 1e-12 * (xx + yy + zz).powi(3);
        a >= 0.0
            && b >= 0.0
            && c >= 0.0
            && a * b >= xy * xy
            && a * c >= xz * xz
            && b * c >= yz * yz
            && a * b * c - 2.0 * xy * xz * yz - a * yz * yz - b * xz * xz - c * xy * xy >= -eps
    }
}
