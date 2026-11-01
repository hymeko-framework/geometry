//! Force transmissions projected from resolved HyMeKo incidences: two-attachment drives, and
//! hydraulic braided muscles whose path runs through ordered via points.
//! Reading them out of HyMeKo incidences is transformation's `hymeko_geometry`.
//! These attach to the existing body model; they introduce no pose tree or IR. An attachment is
//! either a child of its link or a `mount` hyperedge onto it (an actuation layer that leaves a
//! shared body unedited); a drive belongs to a robot when declared under its root or when its
//! path runs between that robot's attachments.

use crate::handles::Handles;

#[derive(Clone, Debug)]
pub struct Attachment<H: Handles> {
    pub did: H::Id,
    pub name: String,
    pub link: H::Id,
    pub position: [f64; 3],
    /// Optional spherical tendon guide, rigidly attached to this link. Mass is part of the link inertia.
    pub radius_m: Option<f64>,
}

#[derive(Clone, Debug)]
pub enum DriveLaw {
    /// MuJoCo-compatible Hill-type force-length/velocity law with activation dynamics.
    Muscle {
        force: f64,
        activation_s: f64,
        deactivation_s: f64,
    },
    /// Equal-area double-acting cylinder: filtered differential pressure times area.
    /// No fluid circuit, valve flow or bulk-modulus model is implied.
    Hydraulic {
        area_m2: f64,
        pressure_pa: f64,
        response_s: f64,
        damping_ns_m: f64,
    },
    /// Hydraulic braided (McKibben-type) muscle: the pressurised sleeve is the contractile element,
    /// in series with a tendon spring and in parallel with a passive spring and damper (the
    /// three-element Hill arrangement). The valve lags the commanded pressure by `response_s`.
    /// Pull only. Simulated by Kawakern (`engine::muscle`); MJCF has no such element.
    HydraulicMuscle {
        pressure_pa: f64,
        response_s: f64,
        sleeve: BraidedSleeve,
        tendon_stiffness_n_m: f64,
        tendon_slack_m: f64,
        parallel_stiffness_n_m: f64,
        parallel_rest_m: f64,
        damping_ns_m: f64,
    },
}

/// The declared geometry of a braided muscle sleeve. Inextensible threads of length `b` wound
/// `n` turns fix the sleeve's shape by its length alone: at length `x`, `cos(theta) = x / b`
/// and the diameter is `b sin(theta) / (n pi)`. Pressure shortens the sleeve and so widens it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BraidedSleeve {
    pub braid_length_m: f64,
    pub turns: f64,
}

impl BraidedSleeve {
    /// Braid angle `theta` at sleeve length `length` (clamped to the braid's reach).
    pub fn braid_angle(&self, length: f64) -> f64 {
        (length / self.braid_length_m).clamp(0.0, 1.0).acos()
    }

    /// Sleeve diameter at length `length`.
    pub fn diameter(&self, length: f64) -> f64 {
        self.braid_length_m * self.braid_angle(length).sin() / (self.turns * std::f64::consts::PI)
    }

    /// Pulling force per unit pressure, `b^2 / (4 pi n^2) (3 cos^2 theta - 1)` (Chou and
    /// Hannaford's ideal braided muscle), zero once the braid passes `acos(1 / sqrt 3)`.
    pub fn force_per_pressure(&self, length: f64) -> f64 {
        let c = (length / self.braid_length_m).clamp(0.0, 1.0);
        let b = self.braid_length_m;
        (b * b / (4.0 * std::f64::consts::PI * self.turns * self.turns) * (3.0 * c * c - 1.0)).max(0.0)
    }

    /// Shortest length at which the sleeve still pulls: `b / sqrt 3`.
    pub fn shortest_pulling_length(&self) -> f64 {
        self.braid_length_m / 3f64.sqrt()
    }
}

#[derive(Clone, Debug)]
pub struct LinearDrive<H: Handles> {
    pub did: H::Id,
    pub name: String,
    pub origin: H::Id,
    /// Ordered via points between origin and insertion (hydraulic muscles only); empty otherwise.
    pub via: Vec<H::Id>,
    pub insertion: H::Id,
    /// Optional spherical guide and its preferred-side attachment, both resolved declarations.
    pub wrapping: Option<[H::Id; 2]>,
    /// Explicit control coordinate; geometry, not this reference, determines the moment arm.
    pub joint: H::Id,
    pub length_range_m: [f64; 2],
    pub radius_m: f64,
    pub law: DriveLaw,
}

#[derive(Clone, Debug, Default)]
pub struct Actuation<H: Handles> {
    pub attachments: Vec<Attachment<H>>,
    pub drives: Vec<LinearDrive<H>>,
    /// Malformed authored connections must fail projection, never disappear silently.
    pub errors: Vec<String>,
}
