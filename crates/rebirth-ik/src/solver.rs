//! FABRIK for whole bone trees with several effectors.
//!
//! The solver works on global node positions: a *forward* pass moves every chain tip onto its
//! target and drags the chain behind it; where chains meet, the pulled-to positions of all
//! children are averaged. A *backward* pass then pins the bases and re-extends every chain
//! outwards, which restores the bone lengths. Optionally joint rotations are reconstructed from
//! the change of every segment, and joint constraints are enforced in the backward pass.

use alloc::vec::Vec;

use crate::chain::ChainTree;
use crate::constraint::Constraint;
use crate::math::{Quat, Vec3};
use crate::skeleton::{Effector, NodeId, Skeleton};

/// Solver settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    /// Upper bound of forward/backward sweeps.
    pub max_iterations: u32,
    /// Distance at which an effector counts as having reached its target.
    pub tolerance: f32,
    /// Stop as soon as every effector is within `tolerance`. Disable to always run
    /// `max_iterations` sweeps (useful for benchmarks and for fixed-cost animation budgets).
    pub early_exit: bool,
    /// Reconstruct local rotations of the solved joints (otherwise only positions change).
    pub joint_rotations: bool,
    /// Enforce [`Constraint`]s attached to nodes.
    pub constraints: bool,
    /// With joint rotations: blend the effector node's rotation towards the effector's `target_rotation`.
    pub target_rotations: bool,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            max_iterations: 20,
            tolerance: 1e-3,
            early_exit: true,
            joint_rotations: false,
            constraints: false,
            target_rotations: false,
        }
    }
}

/// What happened during [`Fabrik::solve`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SolveInfo {
    /// Sweeps that were executed.
    pub iterations: u32,
    /// All effectors ended within tolerance of their (weighted) targets.
    pub converged: bool,
    /// Largest remaining distance between an effector node and its target.
    pub max_error: f32,
}

const NO_PARENT: u32 = u32::MAX;

#[derive(Default)]
struct Scratch {
    pos: Vec<Vec3>,
    rot: Vec<Quat>,
    init_pos: Vec<Vec3>,
    init_rot: Vec<Quat>,
    len: Vec<f32>,
    target: Vec<Vec3>,
    sparent: Vec<u32>,
    constraint: Vec<Constraint>,
    delta: Vec<Quat>,
    gp: Vec<Vec3>,
    gr: Vec<Quat>,
    in_chain: Vec<bool>,
    is_effector: Vec<bool>,
}

/// Reusable FABRIK solver bound to the *shape* of a skeleton (which nodes have effectors, chain lengths).
/// Call [`rebuild`](Fabrik::rebuild) after you add/remove effectors, change `chain_length`, or add nodes.
pub struct Fabrik {
    pub config: Config,
    tree: ChainTree,
    s: Scratch,
    effectors: Vec<NodeId>,
}

impl Fabrik {
    pub fn new(skel: &Skeleton, config: Config) -> Self {
        let mut f = Fabrik {
            config,
            tree: ChainTree::default(),
            s: Scratch::default(),
            effectors: Vec::new(),
        };
        f.rebuild(skel);
        f
    }

    /// The chains currently solved. Handy for debugging and visualisation.
    pub fn chains(&self) -> &ChainTree {
        &self.tree
    }

    /// Rebuilds the chain forest from the skeleton's effectors.
    pub fn rebuild(&mut self, skel: &Skeleton) {
        self.tree = ChainTree::build(skel);
        self.effectors.clear();
        for (i, n) in skel.nodes().iter().enumerate() {
            if n.effector.is_some() {
                self.effectors.push(NodeId(i as u32));
            }
        }
        let n = skel.len();
        let s = &mut self.s;
        s.sparent.clear();
        s.sparent.extend(
            skel.nodes()
                .iter()
                .map(|n| n.parent().map_or(NO_PARENT, |p| p.0)),
        );
        s.in_chain.clear();
        s.in_chain.resize(n, false);
        for c in &self.tree.chains {
            for id in &c.nodes {
                s.in_chain[id.index()] = true;
            }
        }
        s.is_effector.clear();
        s.is_effector.resize(n, false);
        for id in &self.effectors {
            s.is_effector[id.index()] = true;
        }
    }

    /// Runs the solver and writes the solved *local* transforms back into `skel`.
    ///
    /// Bone lengths are taken from the skeleton's local offsets at call time.
    pub fn solve(&mut self, skel: &mut Skeleton) -> SolveInfo {
        let n = skel.len();
        debug_assert_eq!(
            self.s.sparent.len(),
            n,
            "skeleton changed shape: call rebuild()"
        );
        let cfg = self.config;
        let s = &mut self.s;

        skel.global_transforms(&mut s.pos, &mut s.rot);
        s.len.clear();
        s.len
            .extend(skel.nodes().iter().map(|n| n.position.length()));
        s.constraint.clear();
        s.constraint
            .extend(skel.nodes().iter().map(|n| n.constraint));
        s.init_pos.clear();
        s.init_pos.extend_from_slice(&s.pos);
        s.init_rot.clear();
        s.init_rot.extend_from_slice(&s.rot);
        s.target.clear();
        s.target.resize(n, Vec3::ZERO);

        // Effective targets (weight, optional nlerp around the chain base).
        for c in &self.tree.chains {
            let tip = c.tip();
            if let Some(e) = &skel.node(tip).effector {
                s.target[tip.index()] =
                    effective_target(e, s.pos[tip.index()], s.pos[c.base().index()]);
            }
        }

        let tol2 = cfg.tolerance * cfg.tolerance;
        let mut iterations = 0;
        let mut converged = effectors_within(&self.tree, s, tol2);
        while iterations < cfg.max_iterations && !(cfg.early_exit && converged) {
            for &r in &self.tree.roots {
                forward(&self.tree, s, r);
                let base = self.tree.chains[r].base();
                let bp = s.pos[base.index()];
                backward(&self.tree, s, r, bp, cfg.constraints);
            }
            iterations += 1;
            converged = effectors_within(&self.tree, s, tol2);
        }

        let mut max_error = 0.0f32;
        for c in &self.tree.chains {
            if skel.node(c.tip()).effector.is_some() {
                max_error =
                    max_error.max(s.pos[c.tip().index()].distance(s.target[c.tip().index()]));
            }
        }

        if cfg.joint_rotations {
            joint_rotations(&self.tree, s, skel, &cfg);
        }
        write_back(s, skel, cfg.joint_rotations);

        SolveInfo {
            iterations,
            converged: max_error <= cfg.tolerance,
            max_error,
        }
    }
}

fn effective_target(e: &Effector, node_pos: Vec3, base_pos: Vec3) -> Vec3 {
    let mut t = node_pos + (e.target_position - node_pos) * e.weight;
    if e.flags
        .contains(crate::skeleton::EffectorFlags::WEIGHT_NLERP)
        && e.weight < 1.0
    {
        let to_target = (e.target_position - base_pos).length() * e.weight;
        let to_node = (node_pos - base_pos).length() * (1.0 - e.weight);
        t = base_pos + (t - base_pos).normalized() * (to_target + to_node);
    }
    t
}

fn effectors_within(tree: &ChainTree, s: &Scratch, tol2: f32) -> bool {
    tree.chains.iter().all(|c| {
        let t = c.tip().index();
        !s.is_effector[t] || (s.pos[t] - s.target[t]).length_squared() <= tol2
    })
}

#[inline]
fn direction_to(from: Vec3, to: Vec3) -> Vec3 {
    let d = to - from;
    if d.length_squared() > 1e-18 {
        d.normalized()
    } else {
        Vec3::Y
    }
}

/// Moves the chain tips onto their targets and drags the chains behind. Returns the position
/// the chain's base node was pulled to.
fn forward(tree: &ChainTree, s: &mut Scratch, ci: usize) -> Vec3 {
    let chain = &tree.chains[ci];
    let tip = chain.nodes[0].index();
    let mut sum = Vec3::ZERO;
    let mut count = 0.0f32;
    for &child in &chain.children {
        sum += forward(tree, s, child);
        count += 1.0;
    }
    // A tip with an effector contributes its own target (also when child chains meet here).
    if s.is_effector[tip] {
        sum += s.target[tip];
        count += 1.0;
    }
    let mut target = if count > 0.0 { sum / count } else { s.pos[tip] };

    for w in chain.nodes.windows(2) {
        let (child, parent) = (w[0].index(), w[1].index());
        s.pos[child] = target;
        // The parent should sit `len[child]` away from the child, towards where it currently is.
        target = s.pos[child] + direction_to(s.pos[child], s.pos[parent]) * s.len[child];
    }
    target
}

/// Pins the base and re-extends the chain towards the tip, then recurses into child chains.
fn backward(tree: &ChainTree, s: &mut Scratch, ci: usize, base_pos: Vec3, constraints: bool) {
    let chain = &tree.chains[ci];
    let k = chain.nodes.len() - 1;
    s.pos[chain.nodes[k].index()] = base_pos;
    for i in (0..k).rev() {
        let (child, parent) = (chain.nodes[i].index(), chain.nodes[i + 1].index());
        let mut dir = direction_to(s.pos[parent], s.pos[child]);
        if constraints && s.constraint[parent] != Constraint::None {
            let g = s.sparent[parent];
            if g != NO_PARENT {
                let prev = direction_to(s.pos[g as usize], s.pos[parent]);
                dir = s.constraint[parent].limit(prev, dir);
            }
        }
        s.pos[child] = s.pos[parent] + dir * s.len[child];
    }
    let tip_pos = s.pos[chain.nodes[0].index()];
    for &c in &chain.children {
        backward(tree, s, c, tip_pos, constraints);
    }
}

/// Derives global joint rotations from how every segment turned between the rest pose and the solution.
fn joint_rotations(tree: &ChainTree, s: &mut Scratch, skel: &Skeleton, cfg: &Config) {
    let n = s.pos.len();
    s.delta.clear();
    s.delta.resize(n, Quat::IDENTITY);
    for &r in &tree.roots {
        delta_chain(tree, s, skel, r, cfg);
    }
    // global solved rotation = delta * initial rotation (for nodes in chains)
    for i in 0..n {
        if s.in_chain[i] {
            s.rot[i] = s.delta[i] * s.init_rot[i];
        }
    }
}

fn delta_chain(tree: &ChainTree, s: &mut Scratch, skel: &Skeleton, ci: usize, cfg: &Config) {
    let chain = &tree.chains[ci];
    let tip = chain.nodes[0].index();
    // Children first: the tip they share takes the (sign-aligned) average of their base rotations.
    if !chain.children.is_empty() {
        let (mut ax, mut ay, mut az, mut aw) = (0.0f32, 0.0, 0.0, 0.0);
        let mut first: Option<Quat> = None;
        for &c in &chain.children {
            delta_chain(tree, s, skel, c, cfg);
            let q = s.delta[tree.chains[c].base().index()];
            let q = match first {
                None => {
                    first = Some(q);
                    q
                }
                Some(f) => {
                    if f.dot(q) < 0.0 {
                        Quat::new(-q.x, -q.y, -q.z, -q.w)
                    } else {
                        q
                    }
                }
            };
            ax += q.x;
            ay += q.y;
            az += q.z;
            aw += q.w;
        }
        s.delta[tip] = Quat::new(ax, ay, az, aw).normalized();
    }
    for w in chain.nodes.windows(2) {
        let (child, parent) = (w[0].index(), w[1].index());
        let orig = s.init_pos[child] - s.init_pos[parent];
        let solved = s.pos[child] - s.pos[parent];
        s.delta[parent] = Quat::from_to(orig, solved);
    }
    // The effector node itself has no outgoing segment to compare.
    if chain.children.is_empty() {
        let e = skel.node(chain.tip()).effector.as_ref();
        let inherit = e.is_some_and(|e| {
            e.flags
                .contains(crate::skeleton::EffectorFlags::INHERIT_ROTATION)
        });
        if inherit {
            s.delta[tip] = s.delta[chain.nodes[1].index()];
        }
        if cfg.target_rotations {
            if let Some(e) = e {
                if e.rotation_weight > 0.0 {
                    // Rotation delta that takes the initial orientation towards the requested global rotation.
                    let want = e.target_rotation * s.init_rot[tip].conjugate();
                    s.delta[tip] = s.delta[tip].nlerp(want, e.rotation_weight.clamp(0.0, 1.0));
                }
            }
        }
    }
}

/// Converts the solved global transforms back into local ones. Nodes outside all chains are carried
/// along rigidly by their parents.
fn write_back(s: &mut Scratch, skel: &mut Skeleton, rotations: bool) {
    let n = s.pos.len();
    s.gp.clear();
    s.gp.resize(n, Vec3::ZERO);
    s.gr.clear();
    s.gr.resize(n, Quat::IDENTITY);
    for i in 0..n {
        let solved_rot = if rotations && s.in_chain[i] {
            s.rot[i]
        } else {
            s.init_rot[i]
        };
        let p = s.sparent[i];
        if p == NO_PARENT {
            s.gp[i] = if s.in_chain[i] {
                s.pos[i]
            } else {
                s.init_pos[i]
            };
            s.gr[i] = solved_rot;
            if s.in_chain[i] {
                let node = skel.node_mut(NodeId(i as u32));
                node.position = s.gp[i];
                node.rotation = solved_rot;
            }
            continue;
        }
        let p = p as usize;
        if s.in_chain[i] {
            s.gp[i] = s.pos[i];
            s.gr[i] = solved_rot;
            let inv = s.gr[p].conjugate();
            let node = skel.node_mut(NodeId(i as u32));
            node.position = inv.rotate(s.gp[i] - s.gp[p]);
            node.rotation = (inv * solved_rot).normalized();
        } else {
            let node = skel.node(NodeId(i as u32));
            s.gp[i] = s.gp[p] + s.gr[p].rotate(node.position);
            s.gr[i] = s.gr[p] * node.rotation;
        }
    }
}
