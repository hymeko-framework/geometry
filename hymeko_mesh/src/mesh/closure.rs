//! How a triangle list bounds space, and its edges: the queries a renderer needs to decide whether
//! back faces may be culled and which segments draw its wireframe.
//!
//! Unlike the processing topology, which aliases only bit-identical positions and refuses
//! anything non-manifold, this answers for any list: vertices weld within a small tolerance, and a
//! defect is named rather than raised.
use std::collections::{BTreeSet, HashMap};

/// How a triangle list bounds space once coincident vertices are welded.
///
/// A closed surface wound counter-clockwise outward is an oriented 2-manifold: every edge borders
/// exactly two faces, which walk it in opposite directions. A signed count per edge that merely
/// sums to zero is weaker — two coincident copies of a shell, or four faces fanned around one edge,
/// also sum to zero — so the two directions are counted apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Enclosure {
    /// Every edge is walked once each way and every connected shell encloses positive volume. Seen
    /// from outside, every ray meets one of its front faces before any back face.
    ClosedOutward,
    /// No triangle with three distinct corners, or an index past the vertices.
    Empty,
    /// Some edge borders more than two faces.
    NonManifold,
    /// Some edge borders one face only: the surface has a hole.
    Open,
    /// Some edge's two faces walk it the same way: one of them is wound against its neighbour.
    Misoriented,
    /// Closed, but some shell is wound inward and shows its back faces outside.
    Inward,
}

impl Enclosure {
    /// Vertices weld on a grid of 2^-20 of the list's extent: seam and pole copies that differ by
    /// rounding weld, a real gap does not. A triangle with two welded corners covers nothing and is
    /// skipped. When several defects are present, the gravest is named, in the order of the variants.
    pub fn of(positions: &[[f32; 3]], indices: &[u32]) -> Self {
        let (lo, hi) = positions.iter().fold(([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]), |(lo, hi), p| {
            (std::array::from_fn(|k| lo[k].min(p[k])), std::array::from_fn(|k| hi[k].max(p[k])))
        });
        let extent = (0..3).map(|k| (hi[k] - lo[k]) * (hi[k] - lo[k])).sum::<f32>().sqrt();
        let grid = (extent / (1 << 20) as f32).max(f32::MIN_POSITIVE);
        if !grid.is_finite() {
            return Self::Empty;
        }
        let mut welded = HashMap::with_capacity(positions.len());
        let ids: Vec<u32> = positions
            .iter()
            .map(|p| {
                let next = welded.len() as u32;
                *welded.entry(p.map(|c| (c / grid).round() as i64)).or_insert(next)
            })
            .collect();
        let mut shells = Shells((0..welded.len() as u32).collect());
        let mut faces = Vec::with_capacity(indices.len() / 3);
        // Per edge (low, high), how many faces walk it low to high and how many high to low.
        let mut walks = HashMap::<(u32, u32), [u32; 2]>::with_capacity(indices.len());
        for triangle in indices.chunks_exact(3) {
            let Some(corners) = triangle
                .iter()
                .map(|&i| Some((*ids.get(i as usize)?, positions[i as usize])))
                .collect::<Option<Vec<_>>>()
            else {
                return Self::Empty;
            };
            let [(a, pa), (b, pb), (c, pc)] = [corners[0], corners[1], corners[2]];
            if a == b || b == c || c == a {
                continue;
            }
            for (from, to) in [(a, b), (b, c), (c, a)] {
                walks.entry((from.min(to), from.max(to))).or_default()[usize::from(from > to)] += 1;
            }
            shells.join(a, b);
            shells.join(b, c);
            // Six times the signed volume of the tetrahedron from the origin: summed over a shell,
            // its enclosed volume, positive when wound outward.
            let cross = [pb[1] * pc[2] - pb[2] * pc[1], pb[2] * pc[0] - pb[0] * pc[2], pb[0] * pc[1] - pb[1] * pc[0]];
            faces.push((a, f64::from(pa[0] * cross[0] + pa[1] * cross[1] + pa[2] * cross[2])));
        }
        if faces.is_empty() {
            return Self::Empty;
        }
        let defect = |[up, down]: [u32; 2]| match up + down {
            2 if up == 1 => None,
            1 => Some(Self::Open),
            2 => Some(Self::Misoriented),
            _ => Some(Self::NonManifold),
        };
        // The gravest defect anywhere, independent of the order the edges are visited in.
        if let Some(defect) = walks.values().filter_map(|&counts| defect(counts)).min_by_key(|defect| *defect as u8) {
            return defect;
        }
        let mut volume = HashMap::<u32, f64>::new();
        for (corner, signed) in faces {
            *volume.entry(shells.root(corner)).or_default() += signed;
        }
        if volume.values().all(|&v| v > 0.) { Self::ClosedOutward } else { Self::Inward }
    }
}

/// Connected shells of welded vertices: a disjoint-set forest with path halving.
struct Shells(Vec<u32>);

impl Shells {
    fn root(&mut self, mut i: u32) -> u32 {
        while self.0[i as usize] != i {
            self.0[i as usize] = self.0[self.0[i as usize] as usize];
            i = self.0[i as usize];
        }
        i
    }
    fn join(&mut self, a: u32, b: u32) {
        let (a, b) = (self.root(a), self.root(b));
        self.0[a as usize] = b;
    }
}

/// Every edge of a triangle index list once, as index pairs with the lower index first, in
/// ascending order: the segments of its wireframe. Edges of a repeated index are left out. Indices
/// are compared as given, not welded.
pub fn unique_edges(triangles: &[u32]) -> Vec<u32> {
    triangles
        .chunks_exact(3)
        .flat_map(|t| [[t[0], t[1]], [t[1], t[2]], [t[2], t[0]]])
        .filter(|edge| edge[0] != edge[1])
        .map(|[a, b]| [a.min(b), a.max(b)])
        .collect::<BTreeSet<_>>()
        .into_iter()
        .flatten()
        .collect()
}
