//! Allocation-free FABRIK on a plain slice of points, plus the batch helpers built on it.
//!
//! This is the hot path for crowds, tentacles, ropes and procedural creatures: no skeleton,
//! no heap, `no_std`. `positions[0]` is the fixed base, the last element is the tip.

use crate::math::Vec3;

/// Result of [`solve_chain`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChainResult {
    pub iterations: u32,
    /// Distance between the tip and the target after solving.
    pub error: f32,
}

/// Distances between consecutive points. Writes `positions.len() - 1` values into `lengths`.
pub fn segment_lengths(positions: &[Vec3], lengths: &mut [f32]) {
    for (i, w) in positions.windows(2).enumerate() {
        lengths[i] = w[0].distance(w[1]);
    }
}

/// Solves one open chain with FABRIK. The base (`positions[0]`) never moves and segment lengths
/// are preserved. If the target is out of reach the chain is stretched straight towards it.
///
/// `lengths` must hold `positions.len() - 1` values.
pub fn solve_chain(
    positions: &mut [Vec3],
    lengths: &[f32],
    target: Vec3,
    max_iterations: u32,
    tolerance: f32,
) -> ChainResult {
    let n = positions.len();
    if n < 2 {
        return ChainResult {
            iterations: 0,
            error: 0.0,
        };
    }
    debug_assert!(lengths.len() >= n - 1);
    let base = positions[0];
    let tip = n - 1;

    let total: f32 = lengths[..tip].iter().sum();

    // Out of reach: lay the chain straight along the base → target line (the FABRIK optimum).
    if base.distance(target) > total * (1.0 + 1e-6) {
        let dir = (target - base).normalized();
        let mut p = base;
        for i in 0..tip {
            p += dir * lengths[i];
            positions[i + 1] = p;
        }
        return ChainResult {
            iterations: 1,
            error: positions[tip].distance(target),
        };
    }

    let tol2 = tolerance * tolerance;
    let mut it = 0;
    while it < max_iterations {
        if (positions[tip] - target).length_squared() <= tol2 {
            break;
        }
        // forward: tip → target, drag the rest
        positions[tip] = target;
        for i in (0..tip).rev() {
            let d = positions[i] - positions[i + 1];
            let l2 = d.length_squared();
            let dir = if l2 > 1e-18 {
                d * (1.0 / crate::math::fm::sqrt(l2))
            } else {
                Vec3::Y
            };
            positions[i] = positions[i + 1] + dir * lengths[i];
        }
        // backward: base back to its place, re-extend
        positions[0] = base;
        for i in 0..tip {
            let d = positions[i + 1] - positions[i];
            let l2 = d.length_squared();
            let dir = if l2 > 1e-18 {
                d * (1.0 / crate::math::fm::sqrt(l2))
            } else {
                Vec3::Y
            };
            positions[i + 1] = positions[i] + dir * lengths[i];
        }
        it += 1;
    }
    ChainResult {
        iterations: it,
        error: positions[tip].distance(target),
    }
}

/// A self-contained chain for batch solving.
#[cfg(feature = "std")]
#[derive(Clone, Debug)]
pub struct ChainBuf {
    pub positions: alloc::vec::Vec<Vec3>,
    pub lengths: alloc::vec::Vec<f32>,
    pub target: Vec3,
}

#[cfg(feature = "std")]
impl ChainBuf {
    /// Chain whose segment lengths are taken from the initial `positions`.
    pub fn new(positions: alloc::vec::Vec<Vec3>, target: Vec3) -> Self {
        let mut lengths = alloc::vec![0.0; positions.len().saturating_sub(1)];
        segment_lengths(&positions, &mut lengths);
        ChainBuf {
            positions,
            lengths,
            target,
        }
    }
}

/// Solves many independent chains one after another on the calling thread.
#[cfg(feature = "std")]
pub fn solve_chains(chains: &mut [ChainBuf], max_iterations: u32, tolerance: f32) {
    for c in chains.iter_mut() {
        solve_chain(
            &mut c.positions,
            &c.lengths,
            c.target,
            max_iterations,
            tolerance,
        );
    }
}

/// Solves many independent chains in parallel with rayon.
#[cfg(feature = "rayon")]
pub fn solve_chains_par(chains: &mut [ChainBuf], max_iterations: u32, tolerance: f32) {
    use rayon::prelude::*;
    chains.par_iter_mut().for_each(|c| {
        solve_chain(
            &mut c.positions,
            &c.lengths,
            c.target,
            max_iterations,
            tolerance,
        );
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    fn straight(n: usize) -> Vec<Vec3> {
        (0..n).map(|i| Vec3::new(0.0, i as f32, 0.0)).collect()
    }

    fn lengths_ok(p: &[Vec3], l: &[f32]) -> bool {
        p.windows(2)
            .zip(l)
            .all(|(w, &l)| (w[0].distance(w[1]) - l).abs() < 1e-3 * (1.0 + l))
    }

    #[test]
    fn reaches_a_reachable_target() {
        let mut p = straight(8);
        let mut l = [0.0; 7];
        segment_lengths(&p, &mut l);
        let target = Vec3::new(3.0, 2.5, -1.0);
        let r = solve_chain(&mut p, &l, target, 100, 1e-4);
        assert!(r.error < 1e-3, "error {}", r.error);
        assert!(lengths_ok(&p, &l));
        assert_eq!(p[0], Vec3::ZERO, "base must stay put");
    }

    #[test]
    fn stretches_towards_unreachable_target() {
        let mut p = straight(5); // total length 4
        let mut l = [0.0; 4];
        segment_lengths(&p, &mut l);
        let target = Vec3::new(100.0, 0.0, 0.0);
        solve_chain(&mut p, &l, target, 20, 1e-3);
        assert!((p[4] - Vec3::new(4.0, 0.0, 0.0)).length() < 1e-4);
        assert!(lengths_ok(&p, &l));
    }

    #[test]
    fn already_solved_takes_zero_iterations() {
        let mut p = straight(4);
        let mut l = [0.0; 3];
        segment_lengths(&p, &mut l);
        let tip = p[3];
        let r = solve_chain(&mut p, &l, tip, 20, 1e-3);
        assert_eq!(r.iterations, 0);
    }

    #[test]
    fn single_point_is_a_noop() {
        let mut p = [Vec3::ZERO];
        let r = solve_chain(&mut p, &[], Vec3::X, 10, 1e-3);
        assert_eq!(r.iterations, 0);
    }

    #[cfg(feature = "std")]
    #[test]
    fn batch_matches_single() {
        let mut a = alloc::vec![
            ChainBuf::new(straight(6), Vec3::new(2.0, 2.0, 1.0)),
            ChainBuf::new(straight(10), Vec3::new(-3.0, 4.0, 0.5))
        ];
        let mut b = a.clone();
        solve_chains(&mut a, 30, 1e-4);
        for c in b.iter_mut() {
            solve_chain(&mut c.positions, &c.lengths, c.target, 30, 1e-4);
        }
        assert_eq!(a[0].positions, b[0].positions);
        assert_eq!(a[1].positions, b[1].positions);
    }

    #[cfg(feature = "rayon")]
    #[test]
    fn parallel_equals_serial() {
        let mk = |i: usize| {
            ChainBuf::new(
                straight(9),
                Vec3::new(
                    (i % 7) as f32 - 3.0,
                    3.0 + (i % 3) as f32,
                    (i % 5) as f32 * 0.3,
                ),
            )
        };
        let mut s: Vec<ChainBuf> = (0..200).map(mk).collect();
        let mut p = s.clone();
        solve_chains(&mut s, 25, 1e-4);
        solve_chains_par(&mut p, 25, 1e-4);
        for (a, b) in s.iter().zip(&p) {
            assert_eq!(a.positions, b.positions);
        }
    }
}
