//! Where a line meets a solid or crosses a surface, as parameters along the line.
//!
//! A line meets a convex solid in one interval of its parameter or not at all ([`Span`]); a ray
//! from the line's origin enters the solid where that interval starts, or at once when it starts
//! inside. The solids are the unit ball, cube and capped cylinder carried by an affine map, and the
//! capsule — the unit cylinder stretched over a segment, with a ball at each end. A triangle surface
//! is not a volume: a ray meets it where it first crosses a triangle.
//!
//! Points and directions are `[f32; 3]`; a placement is a column-major 4×4 affine matrix, the layout
//! of glam's `Mat4::to_cols_array_2d`.

type V = [f32; 3];

fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: V, b: V) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn scale(a: V, s: f32) -> V {
    [a[0] * s, a[1] * s, a[2] * s]
}

/// The parameters `enter..=exit` for which a line is inside a solid.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Span {
    pub enter: f32,
    pub exit: f32,
}

impl Span {
    /// The whole line.
    pub const WHOLE: Span = Span { enter: f32::NEG_INFINITY, exit: f32::INFINITY };

    /// `None` when empty, and when either end is NaN.
    pub fn new(enter: f32, exit: f32) -> Option<Span> {
        (enter <= exit).then_some(Span { enter, exit })
    }

    /// Where the line is inside both solids.
    pub fn and(self, other: Span) -> Option<Span> {
        Span::new(self.enter.max(other.enter), self.exit.min(other.exit))
    }

    /// Where it is inside either. Exact only when the two overlap, as the pieces of one convex
    /// solid do: then the union is again one interval.
    pub fn or(a: Option<Span>, b: Option<Span>) -> Option<Span> {
        match (a, b) {
            (Some(a), Some(b)) => Some(Span { enter: a.enter.min(b.enter), exit: a.exit.max(b.exit) }),
            (a, b) => a.or(b),
        }
    }

    /// The first parameter at or after 0 inside the span: where a ray from the line's origin
    /// enters, 0 when it starts inside, `None` when the span lies behind it.
    pub fn entry(self) -> Option<f32> {
        (self.exit >= 0.).then(|| self.enter.max(0.))
    }
}

/// Where `a·t² + 2b·t + c ≤ 0` with `a ≥ 0`: the inside of a quadric along a line. With `a = 0`
/// the line runs parallel to it and is inside everywhere or nowhere.
fn quadratic(a: f32, b: f32, c: f32) -> Option<Span> {
    if !(a > 0.) {
        return (c <= 0.).then_some(Span::WHOLE);
    }
    let discriminant = b * b - a * c;
    if !(discriminant >= 0.) {
        return None;
    }
    // The root of larger magnitude first, then the other from their product c/a, so neither is
    // the difference of two nearly equal numbers.
    let q = -(b + discriminant.sqrt().copysign(b));
    if q == 0. {
        return Span::new(0., 0.);
    }
    let (t0, t1) = (q / a, c / q);
    Span::new(t0.min(t1), t0.max(t1))
}

/// A unit solid, in its own frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Unit {
    /// `|p| ≤ 1`.
    Ball,
    /// `|x|, |y|, |z| ≤ 1`.
    Cube,
    /// `x² + y² ≤ 1` and `|z| ≤ 1`.
    Cylinder,
}

/// The line `origin + t·step`. Carried into a placed object's own frame, `step` stops being a unit
/// vector but `t` still names the same point, so a parameter found there is a world distance when
/// the original step was a unit direction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Line {
    pub origin: V,
    pub step: V,
}

impl Line {
    pub fn new(origin: V, step: V) -> Self {
        Self { origin, step }
    }

    /// The point at parameter `t`.
    pub fn at(self, t: f32) -> V {
        [self.origin[0] + self.step[0] * t, self.origin[1] + self.step[1] * t, self.origin[2] + self.step[2] * t]
    }

    /// The same line in the frame that the affine `local_from_world` (column-major) maps into.
    pub fn into_frame(self, local_from_world: &[[f32; 4]; 4]) -> Line {
        let m = local_from_world;
        let linear = |v: V| std::array::from_fn(|r| m[0][r] * v[0] + m[1][r] * v[1] + m[2][r] * v[2]);
        let origin: V = linear(self.origin);
        Line { origin: std::array::from_fn(|r| origin[r] + m[3][r]), step: linear(self.step) }
    }

    /// The span inside a unit solid, the line being given in that solid's frame.
    pub fn unit(self, unit: Unit) -> Option<Span> {
        let (o, d) = (self.origin, self.step);
        match unit {
            Unit::Ball => quadratic(dot(d, d), dot(o, d), dot(o, o) - 1.),
            Unit::Cube => self.slab(0)?.and(self.slab(1)?)?.and(self.slab(2)?),
            Unit::Cylinder => {
                quadratic(d[0] * d[0] + d[1] * d[1], o[0] * d[0] + o[1] * d[1], o[0] * o[0] + o[1] * o[1] - 1.)?
                    .and(self.slab(2)?)
            }
        }
    }

    /// Where `|coordinate axis| ≤ 1`.
    fn slab(self, axis: usize) -> Option<Span> {
        let (o, d) = (self.origin[axis], self.step[axis]);
        if d == 0. {
            return (o.abs() <= 1.).then_some(Span::WHOLE);
        }
        let (a, b) = ((-1. - o) / d, (1. - o) / d);
        Span::new(a.min(b), a.max(b))
    }

    /// Where the line is within `radius` of `centre`: the unit ball, moved and scaled.
    pub fn ball(self, centre: V, radius: f32) -> Option<Span> {
        Line { origin: scale(sub(self.origin, centre), 1. / radius), step: scale(self.step, 1. / radius) }.unit(Unit::Ball)
    }

    /// Where it is within `radius` of the segment `a`–`b`: the unit cylinder stretched over the
    /// segment, and a ball at each end.
    pub fn capsule(self, a: V, b: V, radius: f32) -> Option<Span> {
        let caps = Span::or(self.ball(a, radius), self.ball(b, radius));
        let half = scale(sub(b, a), 0.5);
        let length = dot(half, half).sqrt();
        if !(length > 0.) {
            return caps;
        }
        let z = scale(half, 1. / length);
        let (x, y) = orthonormal_pair(z);
        // The frame's axes are orthogonal, so its inverse projects onto them and divides by their scales.
        let into = |v: V| [dot(v, x) / radius, dot(v, y) / radius, dot(v, z) / length];
        let centre = [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5];
        let body = Line { origin: into(sub(self.origin, centre)), step: into(self.step) }.unit(Unit::Cylinder);
        Span::or(body, caps)
    }

    /// The least `t ≥ 0` at which the line crosses a triangle of `indices` over `positions`, from
    /// either side (Möller–Trumbore). Triangles naming a missing vertex are skipped.
    pub fn first_crossing(self, positions: &[V], indices: &[u32]) -> Option<f32> {
        let corner = |i: u32| positions.get(i as usize).copied();
        let mut first: Option<f32> = None;
        for triangle in indices.chunks_exact(3) {
            let (Some(a), Some(b), Some(c)) = (corner(triangle[0]), corner(triangle[1]), corner(triangle[2])) else {
                continue;
            };
            let (ab, ac) = (sub(b, a), sub(c, a));
            let p = cross(self.step, ac);
            let det = dot(ab, p);
            if det == 0. || !det.is_finite() {
                continue; // the line runs in the triangle's plane: it does not cross it
            }
            let s = sub(self.origin, a);
            let u = dot(s, p) / det;
            let q = cross(s, ab);
            let v = dot(self.step, q) / det;
            if u < 0. || v < 0. || u + v > 1. {
                continue;
            }
            let t = dot(ac, q) / det;
            if t >= 0. && first.is_none_or(|f| t < f) {
                first = Some(t);
            }
        }
        first
    }
}

/// Two unit vectors completing the unit `z` to a right-handed orthonormal frame (Duff et al. 2017).
fn orthonormal_pair(z: V) -> (V, V) {
    let sign = 1.0f32.copysign(z[2]);
    let a = -1.0 / (sign + z[2]);
    let b = z[0] * z[1] * a;
    ([1.0 + sign * z[0] * z[0] * a, sign * b, -sign * z[0]], [b, sign + z[1] * z[1] * a, -z[1]])
}
