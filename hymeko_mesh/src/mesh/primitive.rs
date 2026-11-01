//! Unit primitives, the shapes a scene draws by scaling and placing them: the cube `[-1, 1]³`, the
//! quad `[-1, 1]²` at `z = 0` facing `+z`, the unit UV sphere and the capped unit cylinder along `z`
//! (`|z| ≤ 1`). Each has one exact level of detail and a unit normal at every vertex.
use super::{MeshLod, TriangleMesh};
use std::f32::consts::PI;

fn surface(
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    uv: Vec<[f32; 2]>,
    tangents: Vec<[f32; 4]>,
    indices: Vec<u32>,
) -> TriangleMesh {
    TriangleMesh { positions, normals, lods: vec![MeshLod { indices, error: 0.0 }], uv, tangents }
}

/// The cube, four corners per face so that each face keeps its own normal, its texture square and
/// a tangent along its first edge. Counter-clockwise seen from outside.
pub fn cube() -> TriangleMesh {
    const FACES: [([f32; 3], [[f32; 3]; 4]); 6] = [
        ([0., 0., 1.], [[-1., -1., 1.], [1., -1., 1.], [1., 1., 1.], [-1., 1., 1.]]),
        ([0., 0., -1.], [[1., -1., -1.], [-1., -1., -1.], [-1., 1., -1.], [1., 1., -1.]]),
        ([1., 0., 0.], [[1., -1., 1.], [1., -1., -1.], [1., 1., -1.], [1., 1., 1.]]),
        ([-1., 0., 0.], [[-1., -1., -1.], [-1., -1., 1.], [-1., 1., 1.], [-1., 1., -1.]]),
        ([0., 1., 0.], [[-1., 1., 1.], [1., 1., 1.], [1., 1., -1.], [-1., 1., -1.]]),
        ([0., -1., 0.], [[-1., -1., -1.], [1., -1., -1.], [1., -1., 1.], [-1., -1., 1.]]),
    ];
    let (mut positions, mut normals, mut uv, mut tangents, mut indices) = (vec![], vec![], vec![], vec![], vec![]);
    for (normal, corners) in FACES {
        let base = positions.len() as u32;
        let edge: [f32; 3] = std::array::from_fn(|k| corners[1][k] - corners[0][k]);
        let recip = 1.0 / edge.iter().map(|v| v * v).sum::<f32>().sqrt();
        let tangent = [edge[0] * recip, edge[1] * recip, edge[2] * recip, 1.0];
        for (corner, texel) in corners.into_iter().zip([[0., 0.], [1., 0.], [1., 1.], [0., 1.]]) {
            positions.push(corner);
            normals.push(normal);
            uv.push(texel);
            tangents.push(tangent);
        }
        indices.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    surface(positions, normals, uv, tangents, indices)
}

/// The quad, facing `+z`, counter-clockwise seen from there. One-sided and open: a floor.
pub fn quad() -> TriangleMesh {
    let positions = vec![[-1., -1., 0.], [1., -1., 0.], [1., 1., 0.], [-1., 1., 0.]];
    surface(positions, vec![[0., 0., 1.]; 4], vec![], vec![], vec![0, 1, 2, 0, 2, 3])
}

/// The unit sphere in `sectors` around `z` (at least 3) and `stacks` from pole to pole (at least
/// 2): normals equal to positions, texture coordinates over the longitude–latitude square, tangents
/// along the parallels. The seam column and the pole rows repeat positions, so the pole rows carry
/// zero-area triangles; welding them closes the surface.
pub fn uv_sphere(sectors: u32, stacks: u32) -> TriangleMesh {
    let (sectors, stacks) = (sectors.max(3), stacks.max(2));
    let (mut positions, mut uv, mut tangents, mut indices) = (vec![], vec![], vec![], vec![]);
    for i in 0..=stacks {
        let phi = PI * (i as f32) / (stacks as f32);
        let (sp, cp) = phi.sin_cos();
        for j in 0..=sectors {
            let th = 2.0 * PI * (j as f32) / (sectors as f32);
            let (st, ct) = th.sin_cos();
            positions.push([sp * ct, sp * st, cp]);
            uv.push([j as f32 / sectors as f32, i as f32 / stacks as f32]);
            tangents.push([-st, ct, 0., 1.]);
        }
    }
    let row = sectors + 1;
    for i in 0..stacks {
        for j in 0..sectors {
            let a = i * row + j;
            let b = a + row;
            indices.extend([a, b, a + 1, a + 1, b, b + 1]);
        }
    }
    let normals = positions.clone();
    surface(positions, normals, uv, tangents, indices)
}

/// The cylinder of radius 1 between `z = -1` and `z = 1` in `segments` around (at least 3): a
/// side with outward radial normals, and two caps fanned from their centres with axial normals,
/// all counter-clockwise seen from outside. (Kawamirai's generator, which this replaces, wound the
/// side the other way, which a front-culled shadow pass rendered from the lit side.)
pub fn cylinder(segments: u32) -> TriangleMesh {
    let seg = segments.max(3);
    let (mut positions, mut normals, mut indices) = (vec![], vec![], vec![]);
    for j in 0..=seg {
        let th = 2.0 * PI * (j as f32) / (seg as f32);
        let (s, c) = th.sin_cos();
        positions.extend([[c, s, -1.0], [c, s, 1.0]]);
        normals.extend([[c, s, 0.0], [c, s, 0.0]]);
    }
    // Bottom a and top a + 1 at one angle, a + 2 and a + 3 at the next: counter-clockwise seen
    // from outside, like the caps, so the surface is closed outward.
    for j in 0..seg {
        let a = j * 2;
        indices.extend([a, a + 2, a + 1, a + 1, a + 2, a + 3]);
    }
    for (zc, nz) in [(-1.0f32, [0.0, 0.0, -1.0f32]), (1.0, [0.0, 0.0, 1.0])] {
        let center = positions.len() as u32;
        positions.push([0.0, 0.0, zc]);
        normals.push(nz);
        let ring = positions.len() as u32;
        for j in 0..=seg {
            let th = 2.0 * PI * (j as f32) / (seg as f32);
            let (s, c) = th.sin_cos();
            positions.push([c, s, zc]);
            normals.push(nz);
        }
        for j in 0..seg {
            if nz[2] > 0.0 {
                indices.extend([center, ring + j, ring + j + 1]);
            } else {
                indices.extend([center, ring + j + 1, ring + j]);
            }
        }
    }
    surface(positions, normals, vec![], vec![], indices)
}
