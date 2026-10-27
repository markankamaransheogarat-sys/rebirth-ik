//! # rebirth-ik
//!
//! Inverse kinematics in safe Rust — a rebirth of the abandoned C library
//! [TheComet/ik](https://github.com/TheComet/ik) (MIT, © 2017 Alex Murray).
//!
//! * [`Skeleton`] + [`Fabrik`]: FABRIK over arbitrary bone *trees* with several effectors,
//!   effector weights, joint constraints and optional joint rotations.
//! * [`solve_chain`]: allocation-free FABRIK for a single chain of points (`no_std`).
//! * [`one_bone`] / [`two_bone`]: closed-form solvers (with pole vector for knees and elbows).
//! * `rayon` feature: [`solve_chains_par`] for crowds.
//!
//! ```
//! use rebirth_ik::*;
//!
//! // a three-bone arm hanging up from the origin
//! let mut skel = Skeleton::new();
//! let root = skel.add_root(Vec3::ZERO);
//! let a = skel.add_child(root, Vec3::new(0.0, 1.0, 0.0));
//! let b = skel.add_child(a, Vec3::new(0.0, 1.0, 0.0));
//! let hand = skel.add_child(b, Vec3::new(0.0, 1.0, 0.0));
//! skel.set_effector(hand, Effector::new(Vec3::new(1.5, 1.5, 0.0)));
//!
//! let mut solver = Fabrik::new(&skel, Config::default());
//! let info = solver.solve(&mut skel);
//! assert!(info.converged);
//! assert!(skel.global_position(hand).distance(Vec3::new(1.5, 1.5, 0.0)) < 1e-2);
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

#[cfg(all(not(feature = "std"), not(feature = "libm")))]
compile_error!(
    "rebirth-ik needs either the `std` feature or the `libm` feature for floating point math"
);

pub mod analytic;
pub mod chain;
pub mod constraint;
pub mod math;
pub mod single;
pub mod skeleton;
pub mod solver;

pub use analytic::{one_bone, segment_rotation, two_bone};
pub use chain::{Chain, ChainTree};
pub use constraint::Constraint;
pub use math::{Quat, Vec3};
#[cfg(feature = "rayon")]
pub use single::solve_chains_par;
pub use single::{segment_lengths, solve_chain, ChainResult};
#[cfg(feature = "std")]
pub use single::{solve_chains, ChainBuf};
pub use skeleton::{Effector, EffectorFlags, Node, NodeId, Skeleton};
pub use solver::{Config, Fabrik, SolveInfo};
