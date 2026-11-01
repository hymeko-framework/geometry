//! The unit primitives are the shapes they claim to be: valid meshes, on their surfaces, with
//! normals pointing away from them.
use hymeko_mesh::mesh::{TriangleMesh, primitive};

fn length(v: [f32; 3]) -> f32 {
    v.iter().map(|x| x * x).sum::<f32>().sqrt()
}

#[test]
fn every_primitive_is_a_valid_mesh_filling_its_unit_bounds() {
    for (name, mesh, expected) in [
        ("cube", primitive::cube(), ([-1.0; 3], [1.0; 3])),
        ("quad", primitive::quad(), ([-1.0, -1.0, 0.0], [1.0, 1.0, 0.0])),
        ("sphere", primitive::uv_sphere(32, 16), ([-1.0; 3], [1.0; 3])),
        ("cylinder", primitive::cylinder(24), ([-1.0; 3], [1.0; 3])),
    ] {
        mesh.validate().unwrap_or_else(|error| panic!("{name}: {error}"));
        let (lo, hi) = mesh.bounds();
        for k in 0..3 {
            assert!((lo[k] - expected.0[k]).abs() < 1e-6 && (hi[k] - expected.1[k]).abs() < 1e-6, "{name} bounds {lo:?}..{hi:?}");
        }
        assert!(mesh.normals.iter().all(|n| (length(*n) - 1.0).abs() < 1e-6), "{name}: unit normals");
    }
}

/// Whether every triangle faces the way its corners' normals point: a winding check that needs
/// no welding.
fn faces_its_normals(mesh: &TriangleMesh) -> bool {
    mesh.lods[0].indices.chunks_exact(3).all(|t| {
        let [a, b, c] = [t[0], t[1], t[2]].map(|i| mesh.positions[i as usize]);
        let (ab, ac) = ([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
        let face = [ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0]];
        let normal = t.iter().fold([0.0; 3], |sum, &i| {
            let n = mesh.normals[i as usize];
            [sum[0] + n[0], sum[1] + n[1], sum[2] + n[2]]
        });
        // A triangle of no appreciable area faces nowhere and passes: the sphere's pole rows, the
        // southern one a ring of radius sin(π) ≈ -8.7e-8 in f32 rather than a point.
        length(face) < 1e-6 || face[0] * normal[0] + face[1] * normal[1] + face[2] * normal[2] > 0.0
    })
}

#[test]
fn every_triangle_faces_the_way_its_normals_point() {
    for (name, mesh) in [
        ("cube", primitive::cube()),
        ("quad", primitive::quad()),
        ("sphere", primitive::uv_sphere(32, 16)),
        ("cylinder", primitive::cylinder(24)),
    ] {
        assert!(faces_its_normals(&mesh), "{name} has a triangle wound against its normals");
    }
}

#[test]
fn the_sphere_lies_on_the_unit_sphere_with_its_texture_and_tangents_along_the_parallels() {
    let sphere = primitive::uv_sphere(24, 12);
    assert_eq!(sphere.positions.len(), 25 * 13);
    for i in 0..sphere.positions.len() {
        let (p, n, uv, t) = (sphere.positions[i], sphere.normals[i], sphere.uv[i], sphere.tangents[i]);
        assert!((length(p) - 1.0).abs() < 1e-6 && p == n, "on the sphere, normal outward");
        assert!(uv.iter().all(|c| (0.0..=1.0).contains(c)));
        assert!((t[0] * n[0] + t[1] * n[1] + t[2] * n[2]).abs() < 1e-6 && t[2] == 0.0 && t[3] == 1.0, "tangent along a parallel");
    }
    let least = primitive::uv_sphere(0, 0);
    assert_eq!(least.positions.len(), 4 * 3, "at least 3 sectors and 2 stacks");
}

#[test]
fn the_cylinder_has_a_radial_side_and_axial_caps() {
    let cylinder = primitive::cylinder(16);
    for (p, n) in cylinder.positions.iter().zip(&cylinder.normals) {
        let radial = (p[0] * p[0] + p[1] * p[1]).sqrt();
        assert!(radial <= 1.0 + 1e-6 && p[2].abs() <= 1.0 + 1e-6);
        let on_side = n[2] == 0.0;
        if on_side {
            assert!((radial - 1.0).abs() < 1e-6 && (n[0] - p[0]).abs() < 1e-6 && (n[1] - p[1]).abs() < 1e-6, "radial");
        } else {
            assert!(n[2] == p[2] && p[2].abs() == 1.0, "a cap's normal points along the axis, out of its end");
        }
    }
}
