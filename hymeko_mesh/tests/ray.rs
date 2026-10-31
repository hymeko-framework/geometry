//! Where a line meets a solid, checked against each solid's definition — which points are inside
//! it — by marching along the line in small steps, over many placements and lines.
use glam::{Mat4, Quat, Vec3};
use hymeko_mesh::mesh::primitive;
use hymeko_mesh::ray::{Line, Span, Unit};

/// Deterministic numbers in -1..1 (xorshift), so a failing case can be replayed.
struct Noise(u64);

impl Noise {
    fn next(&mut self) -> f32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 40) as f32 / (1u64 << 23) as f32 - 1.0
    }
    fn vector(&mut self) -> Vec3 {
        Vec3::new(self.next(), self.next(), self.next())
    }
    /// A rotation, a scale of 0.3..1.5 on each axis and a shift: an affine placement.
    fn placement(&mut self) -> Mat4 {
        let rotation = Quat::from_xyzw(self.next(), self.next(), self.next(), self.next()).normalize();
        let scale = (self.vector() + 1.0) * 0.6 + 0.3;
        Mat4::from_scale_rotation_translation(scale, rotation, self.vector())
    }
    /// A unit-speed line from 6 away, aimed near the origin.
    fn line(&mut self) -> Line {
        let origin = self.vector().normalize_or(Vec3::X) * 6.0;
        let direction = (self.vector() * 1.2 - origin).normalize();
        Line::new(origin.to_array(), direction.to_array())
    }
}

/// The unit solid's definition, grown by `margin` (shrunk for a negative one).
fn inside(unit: Unit, p: Vec3, margin: f32) -> bool {
    match unit {
        Unit::Ball => p.length() <= 1.0 + margin,
        Unit::Cube => p.abs().max_element() <= 1.0 + margin,
        Unit::Cylinder => p.truncate().length() <= 1.0 + margin && p.z.abs() <= 1.0 + margin,
    }
}

const STEP: f32 = 0.002;

/// Every march parameter from 0 to 14, `STEP` apart.
fn marched() -> impl Iterator<Item = f32> {
    (0..(14.0 / STEP) as usize).map(|i| i as f32 * STEP)
}

#[test]
fn a_line_enters_every_placed_unit_solid_where_its_definition_says() {
    // The parameter found in the solid's own frame names the same point as in the world, so the
    // world line is marched and each point judged in the unit frame. The entry must be a point of
    // the solid and nothing clearly inside may come before it; a miss must pass nothing clearly
    // inside. The margin absorbs the march's own resolution.
    const MARGIN: f32 = 0.01;
    let mut noise = Noise(0x9e37_79b9_7f4a_7c15);
    for unit in [Unit::Ball, Unit::Cube, Unit::Cylinder] {
        let (mut hits, mut misses) = (0, 0);
        for _ in 0..240 {
            let world_from_unit = noise.placement();
            let unit_from_world = world_from_unit.inverse();
            let line = noise.line();
            let local = |t: f32| unit_from_world.transform_point3(Vec3::from(line.at(t)));
            match line.into_frame(&unit_from_world.to_cols_array_2d()).unit(unit).and_then(Span::entry) {
                Some(entry) => {
                    hits += 1;
                    assert!(inside(unit, local(entry), MARGIN), "{unit:?}: the entry {entry} is not on the solid");
                    let earlier = marched().take_while(|t| *t < entry - STEP).find(|t| inside(unit, local(*t), -MARGIN));
                    assert!(earlier.is_none(), "{unit:?}: inside at {earlier:?}, before the entry {entry}");
                }
                None => {
                    misses += 1;
                    let inside_at = marched().find(|t| inside(unit, local(*t), -MARGIN));
                    assert!(inside_at.is_none(), "{unit:?}: missed, yet inside at {inside_at:?}");
                }
            }
        }
        assert!(hits > 40 && misses > 40, "{unit:?}: {hits} hits, {misses} misses — the lines must cover both");
    }
}

#[test]
fn a_line_enters_a_capsule_at_its_first_point_within_the_radius_of_the_segment() {
    let mut noise = Noise(0x2545_f491_4f6c_dd1d);
    for _ in 0..200 {
        let (a, b, radius) = (noise.vector(), noise.vector() * 1.5, 0.1 + 0.4 * (noise.next() + 1.0));
        let line = noise.line();
        let near = |t: f32| {
            let p = Vec3::from(line.at(t));
            let along = ((p - a).dot(b - a) / (b - a).length_squared()).clamp(0.0, 1.0);
            p.distance(a + (b - a) * along)
        };
        match line.capsule(a.to_array(), b.to_array(), radius).and_then(Span::entry) {
            Some(entry) => {
                assert!(near(entry) <= radius + 1e-3, "the entry {entry} lies {} from the segment", near(entry));
                assert!(marched().take_while(|t| *t < entry - STEP).all(|t| near(t) > radius - 1e-3));
            }
            None => assert!(marched().all(|t| near(t) > radius - 1e-3)),
        }
    }
    // The case a closest-approach formula without re-projection misses: the lines cross outside the
    // segment, at the origin; clamping to the end (1, 1) must be followed by finding the line's
    // nearest point to that end, at distance 1, inside the radius 1.1.
    let line = Line::new([0.0; 3], [1.0, 0.0, 0.0]);
    let entry = line.capsule([1.0, 1.0, 0.0], [2.0, 2.0, 0.0], 1.1).and_then(Span::entry).unwrap();
    assert!((entry - (1.0 - 0.21f32.sqrt())).abs() < 1e-5, "{entry}");
}

#[test]
fn crossing_a_convex_surface_from_outside_is_entering_its_solid() {
    // The cube's triangles and the unit cube are the same boundary: a line from outside first
    // crosses the triangles where it enters the solid.
    let cube = primitive::cube();
    let mut noise = Noise(0x0123_4567_89ab_cdef);
    let mut compared = 0;
    for _ in 0..300 {
        let unit_from_world = noise.placement().inverse().to_cols_array_2d();
        let local = noise.line().into_frame(&unit_from_world);
        if inside(Unit::Cube, Vec3::from(local.origin), 0.0) {
            continue;
        }
        let entered = local.unit(Unit::Cube).and_then(Span::entry);
        let crossed = local.first_crossing(&cube.positions, &cube.lods[0].indices);
        match (entered, crossed) {
            (Some(e), Some(c)) => {
                assert!((e - c).abs() < 1e-4 * e.max(1.0), "entered at {e}, crossed at {c}");
                compared += 1;
            }
            (None, None) => {}
            other => panic!("the solid and its surface disagree: {other:?}"),
        }
    }
    assert!(compared > 50, "only {compared} crossings compared");
}

#[test]
fn a_ray_starting_inside_enters_at_once_and_one_pointing_away_never() {
    let inside_ball = Line::new([0.2, 0.1, 0.0], [1.0, 0.0, 0.0]);
    assert_eq!(inside_ball.unit(Unit::Ball).and_then(Span::entry), Some(0.0));
    let away = Line::new([3.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    assert_eq!(away.unit(Unit::Ball).and_then(Span::entry), None, "the ball lies behind the origin");
    let nan = Line::new([f32::NAN, 0.0, 0.0], [1.0, 0.0, 0.0]);
    assert!(nan.unit(Unit::Cube).is_none() && nan.unit(Unit::Ball).is_none(), "nothing is met along an undefined line");
    let parallel = Line::new([0.5, 0.0, 5.0], [0.0, 0.0, -1.0]);
    assert_eq!(parallel.unit(Unit::Cylinder), Span::new(4.0, 6.0), "along the axis, only the caps bound it");
}
