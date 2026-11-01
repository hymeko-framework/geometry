//! When a triangle list bounds space, and its wireframe edges.
use hymeko_mesh::mesh::closure::{Enclosure, unique_edges};
use hymeko_mesh::mesh::primitive;

/// A cube with split normals around `centre`: four corners per face, every corner position shared
/// by three faces, wound outward or inward.
fn cube(centre: [f32; 3], outward: bool) -> (Vec<[f32; 3]>, Vec<u32>) {
    let (mut positions, mut indices) = (Vec::new(), Vec::new());
    for axis in 0..3 {
        for sign in [1f32, -1.] {
            let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
            let corner = |du: f32, dv: f32| {
                let mut p = centre;
                p[axis] += sign;
                p[u] += du;
                p[v] += dv;
                p
            };
            let base = positions.len() as u32;
            positions.extend([corner(-1., -1.), corner(1., -1.), corner(1., 1.), corner(-1., 1.)]);
            // u × v is +axis, so the face (0, 1, 2) turns outward on the positive side.
            let turn = if (sign > 0.) == outward { [0, 1, 2, 0, 2, 3] } else { [0, 2, 1, 0, 3, 2] };
            indices.extend(turn.map(|i| base + i));
        }
    }
    (positions, indices)
}

/// `positions`/`indices` with `extra` appended, its vertices offset past the first list's.
fn joined(positions: &[[f32; 3]], indices: &[u32], (extra, extra_indices): (Vec<[f32; 3]>, Vec<u32>)) -> (Vec<[f32; 3]>, Vec<u32>) {
    let offset = positions.len() as u32;
    ([positions, &extra].concat(), [indices.to_vec(), extra_indices.iter().map(|i| i + offset).collect()].concat())
}

#[test]
fn a_surface_is_closed_only_when_every_edge_has_one_face_on_each_side() {
    use Enclosure::*;
    let (positions, indices) = cube([0.; 3], true);
    assert_eq!(Enclosure::of(&positions, &indices), ClosedOutward, "the cube: 12 edges, each walked once each way");
    assert_eq!(Enclosure::of(&positions, &indices[..30]), Open, "a missing face leaves its four edges walked one way");
    let mut flipped = indices.clone();
    flipped.swap(1, 2);
    assert_eq!(Enclosure::of(&positions, &flipped), Misoriented, "a face wound against its neighbours walks their edges their way");
    let degenerate = [indices.clone(), vec![0, 0, 1]].concat();
    assert_eq!(Enclosure::of(&positions, &degenerate), ClosedOutward, "a triangle with two welded corners covers nothing");
    assert_eq!(Enclosure::of(&positions, &[0, 0, 1]), Empty);
    assert_eq!(Enclosure::of(&positions, &[0, 1, 99]), Empty, "an index past the vertices");
}

#[test]
fn signed_edge_counts_that_cancel_do_not_make_a_surface() {
    // Each of these balances +1 against -1 on every edge, which a signed sum accepts, yet more
    // than two faces meet at an edge.
    use Enclosure::*;
    let (positions, indices) = cube([0.; 3], true);
    let doubled = joined(&positions, &indices, (positions.clone(), indices.clone()));
    assert_eq!(Enclosure::of(&doubled.0, &doubled.1), NonManifold, "two coincident copies of one shell");
    // Two cubes sharing the edge x = 1, y = 1: four faces fan around it.
    let touching = joined(&positions, &indices, cube([2., 2., 0.], true));
    assert_eq!(Enclosure::of(&touching.0, &touching.1), NonManifold, "two shells meeting along one edge");
}

#[test]
fn every_shell_must_turn_outward() {
    use Enclosure::*;
    let (inward, inward_indices) = cube([0.; 3], false);
    assert_eq!(Enclosure::of(&inward, &inward_indices), Inward, "inward winding shows back faces outside");
    let (positions, indices) = cube([0.; 3], true);
    let both = joined(&positions, &indices, cube([4., 0., 0.], false));
    assert_eq!(Enclosure::of(&both.0, &both.1), Inward, "one inward shell is enough");
    let apart = joined(&positions, &indices, cube([4., 0., 0.], true));
    assert_eq!(Enclosure::of(&apart.0, &apart.1), ClosedOutward, "separate outward shells are each closed");
}

#[test]
fn the_solid_primitives_are_closed_outward_and_the_quad_is_open() {
    for (name, mesh) in [("cube", primitive::cube()), ("sphere", primitive::uv_sphere(96, 48)), ("cylinder", primitive::cylinder(28))] {
        assert_eq!(Enclosure::of(&mesh.positions, &mesh.lods[0].indices), Enclosure::ClosedOutward, "{name}");
    }
    let quad = primitive::quad();
    assert_eq!(Enclosure::of(&quad.positions, &quad.lods[0].indices), Enclosure::Open);
}

#[test]
fn every_wireframe_edge_appears_once_and_a_repeated_index_draws_none() {
    assert_eq!(unique_edges(&[0, 1, 2, 2, 1, 3]), [0, 1, 0, 2, 1, 2, 1, 3, 2, 3]);
    assert_eq!(unique_edges(&[1, 1, 2]), [1, 2]);
    assert!(unique_edges(&[]).is_empty());
    // A closed cube of split faces: 6 faces of 4 corners, each with 4 rim edges and 1 diagonal.
    let cube = primitive::cube();
    assert_eq!(unique_edges(&cube.lods[0].indices).len() / 2, 6 * 5);
}
