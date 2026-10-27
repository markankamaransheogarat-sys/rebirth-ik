//! WebAssembly bindings for the live demo. Everything is 2D on the screen plane (z = 0).

use rebirth_ik::*;
use wasm_bindgen::prelude::*;

/// A ready-made skeleton plus the solver that drives it.
#[wasm_bindgen]
pub struct Rig {
    skel: Skeleton,
    solver: Fabrik,
    effectors: Vec<NodeId>,
    last: SolveInfo,
    cone: Option<f32>,
}

fn v(x: f32, y: f32) -> Vec3 {
    Vec3::new(x, y, 0.0)
}

impl Rig {
    fn finish(skel: Skeleton, effectors: Vec<NodeId>, iterations: u32) -> Rig {
        let solver = Fabrik::new(
            &skel,
            Config {
                max_iterations: iterations,
                tolerance: 0.002,
                ..Config::default()
            },
        );
        Rig {
            skel,
            solver,
            effectors,
            last: SolveInfo {
                iterations: 0,
                converged: true,
                max_error: 0.0,
            },
            cone: None,
        }
    }
}

#[wasm_bindgen]
impl Rig {
    /// A tentacle: `bones` bones of `len` units hanging up from the origin, one effector at the tip.
    pub fn chain(bones: u32, len: f32) -> Rig {
        let mut s = Skeleton::new();
        let mut n = s.add_root(v(0.0, 0.0));
        for _ in 0..bones {
            n = s.add_child(n, v(len, 0.0));
        }
        s.set_effector(n, Effector::new(v(len * bones as f32, 0.0)));
        Rig::finish(s, vec![n], 20)
    }

    /// A humanoid: effectors in order head, left hand, right hand, left foot, right foot.
    /// Hip at the origin, y points up.
    pub fn humanoid(scale: f32) -> Rig {
        let mut s = Skeleton::new();
        let hip = s.add_root(v(0.0, 0.0));
        let mut spine = hip;
        for _ in 0..3 {
            spine = s.add_child(spine, v(0.0, 0.28 * scale));
        }
        let neck = s.add_child(spine, v(0.0, 0.16 * scale));
        let head = s.add_child(neck, v(0.0, 0.18 * scale));
        let limb = |s: &mut Skeleton, from: NodeId, dx: f32, dy: f32, n: u32| {
            let mut c = from;
            for _ in 0..n {
                c = s.add_child(c, v(dx * scale, dy * scale));
            }
            c
        };
        let lh = limb(&mut s, spine, -0.22, -0.02, 3);
        let rh = limb(&mut s, spine, 0.22, -0.02, 3);
        let lf = limb(&mut s, hip, -0.04, -0.3, 3);
        let rf = limb(&mut s, hip, 0.04, -0.3, 3);
        let eff = alloc_effectors(&mut s, &[head, lh, rh, lf, rf]);
        Rig::finish(s, eff, 20)
    }

    /// A spider-like walker: `legs` legs (3 bones each) around a body at the origin.
    /// Effectors are the foot tips, ordered left to right.
    pub fn walker(legs: u32, scale: f32) -> Rig {
        let mut s = Skeleton::new();
        let body = s.add_root(v(0.0, 0.0));
        let mut feet = Vec::new();
        for i in 0..legs {
            let side = if i % 2 == 0 { -1.0 } else { 1.0 };
            // each extra row of legs reaches further out, so the legs of one side do not overlap
            let f = 1.0 + (i / 2) as f32 * 0.45;
            let mut n = s.add_child(body, v(side * 0.08 * scale, 0.0));
            n = s.add_child(n, v(side * 0.24 * f * scale, 0.42 * scale));
            n = s.add_child(n, v(side * 0.3 * f * scale, -0.14 * scale));
            n = s.add_child(n, v(side * 0.2 * f * scale, -0.7 * scale));
            feet.push(n);
        }
        let eff = alloc_effectors(&mut s, &feet);
        Rig::finish(s, eff, 20)
    }

    pub fn effector_count(&self) -> usize {
        self.effectors.len()
    }
    pub fn node_count(&self) -> usize {
        self.skel.len()
    }

    /// Global position of effector `i` as `[x, y]` (before solving this is the rest pose).
    pub fn effector_position(&self, i: usize) -> Vec<f32> {
        let p = self.skel.global_position(self.effectors[i]);
        vec![p.x, p.y]
    }

    /// Moves effector `i`'s target.
    pub fn set_target(&mut self, i: usize, x: f32, y: f32) {
        self.skel.set_target(self.effectors[i], v(x, y));
    }

    /// Moves the root (the immovable base) to `(x, y)`.
    pub fn set_root(&mut self, x: f32, y: f32) {
        self.skel.node_mut(NodeId(0)).position = v(x, y);
    }

    pub fn set_iterations(&mut self, n: u32) {
        self.solver.config.max_iterations = n;
    }

    /// 0 disables constraints. Otherwise every joint that has both a parent and a child bends at most `degrees`.
    pub fn set_cone_degrees(&mut self, degrees: f32) {
        let on = degrees > 0.0;
        self.cone = if on { Some(degrees.to_radians()) } else { None };
        for i in 1..self.skel.len() {
            let id = NodeId(i as u32);
            let has_child = !self.skel.node(id).children().is_empty();
            let c = match self.cone {
                Some(a) if has_child => Constraint::Cone { max_angle: a },
                _ => Constraint::None,
            };
            self.skel.set_constraint(id, c);
        }
        self.solver.config.constraints = on;
    }

    /// Runs the solver. Returns the number of sweeps used.
    pub fn solve(&mut self) -> u32 {
        self.last = self.solver.solve(&mut self.skel);
        self.last.iterations
    }

    /// Runs `reps` full solves with early exit disabled (every solve does all its sweeps) and
    /// restores the setting. Used by the demo to time the solver with a stable, non-zero cost.
    pub fn solve_fixed(&mut self, reps: u32) {
        let early = self.solver.config.early_exit;
        self.solver.config.early_exit = false;
        for _ in 0..reps {
            self.solver.solve(&mut self.skel);
        }
        self.solver.config.early_exit = early;
    }

    /// Largest distance between an effector and its target after the last solve.
    pub fn error(&self) -> f32 {
        self.last.max_error
    }
    pub fn converged(&self) -> bool {
        self.last.converged
    }

    /// Global positions of all nodes as `[x0, y0, x1, y1, ...]`.
    pub fn positions(&self) -> Vec<f32> {
        let (mut p, mut r) = (Vec::new(), Vec::new());
        self.skel.global_transforms(&mut p, &mut r);
        p.iter().flat_map(|q| [q.x, q.y]).collect()
    }

    /// Parent index of every node (`u32::MAX` for the root), to draw bones.
    pub fn parents(&self) -> Vec<u32> {
        self.skel
            .nodes()
            .iter()
            .map(|n| n.parent().map_or(u32::MAX, |p| p.0))
            .collect()
    }

    /// Indices of the effector nodes.
    pub fn effector_nodes(&self) -> Vec<u32> {
        self.effectors.iter().map(|n| n.0).collect()
    }
}

fn alloc_effectors(s: &mut Skeleton, nodes: &[NodeId]) -> Vec<NodeId> {
    let (mut p, mut r) = (Vec::new(), Vec::new());
    s.global_transforms(&mut p, &mut r);
    for &n in nodes {
        s.set_effector(n, Effector::new(p[n.index()]));
    }
    nodes.to_vec()
}

/// Two-bone analytic solve. Returns `[midX, midY, tipX, tipY]`.
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)] // flat scalar arguments are the cheapest JS <-> WASM interface
pub fn two_bone_solve(
    bx: f32,
    by: f32,
    upper: f32,
    lower: f32,
    tx: f32,
    ty: f32,
    px: f32,
    py: f32,
) -> Vec<f32> {
    let base = v(bx, by);
    let mid = base + v(upper, 0.0);
    let tip = mid + v(lower, 0.0);
    let (m, t) = two_bone(base, mid, tip, v(tx, ty), Some(v(px, py)));
    vec![m.x, m.y, t.x, t.y]
}

/// Solves `n` independent 10-bone chains (the "crowd" benchmark) and returns a checksum so the
/// work cannot be optimized away. Time it from JavaScript with `performance.now()`.
#[wasm_bindgen]
pub fn crowd_benchmark(n: u32, sweeps: u32) -> f32 {
    let mut sum = 0.0f32;
    let pts: Vec<Vec3> = (0..11).map(|i| v(0.0, i as f32)).collect();
    let mut lengths = [0.0f32; 10];
    segment_lengths(&pts, &mut lengths);
    let mut work = pts.clone();
    for i in 0..n {
        let f = i as f32;
        let target = Vec3::new(
            (f * 0.37).sin() * 6.0,
            3.0 + (f * 0.11).cos() * 3.0,
            (f * 0.53).sin() * 6.0,
        );
        work.copy_from_slice(&pts);
        solve_chain(&mut work, &lengths, target, sweeps, 0.0);
        sum += work[10].x + work[10].y + work[10].z;
    }
    sum
}

/// Library version string.
#[wasm_bindgen]
pub fn version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}
