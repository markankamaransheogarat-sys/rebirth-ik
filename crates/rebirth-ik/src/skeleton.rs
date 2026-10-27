//! The bone tree: nodes with local transforms, optional effectors and constraints.

use alloc::vec::Vec;

use crate::constraint::Constraint;
use crate::math::{Quat, Vec3};

/// Index of a node inside a [`Skeleton`]. A parent always has a smaller id than its children.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct NodeId(pub u32);

impl NodeId {
    #[inline(always)]
    pub fn index(self) -> usize {
        self.0 as usize
    }
}

/// Behaviour switches of an [`Effector`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EffectorFlags(pub u8);

impl EffectorFlags {
    /// Intermediate weights rotate the target around the chain base instead of
    /// interpolating linearly (nicer when the solved pose differs a lot from the rest pose).
    pub const WEIGHT_NLERP: u8 = 0x01;
    /// The effector node copies the rotation of its parent (otherwise it keeps its orientation).
    pub const INHERIT_ROTATION: u8 = 0x02;

    pub const fn contains(self, bit: u8) -> bool {
        self.0 & bit != 0
    }
}

/// Tells the solver where a node should go.
#[derive(Clone, Debug, PartialEq)]
pub struct Effector {
    /// Global position the node should reach.
    pub target_position: Vec3,
    /// Global rotation the node should take (used with joint rotations).
    pub target_rotation: Quat,
    /// 0.0 = the solver ignores this effector, 1.0 = it fully tries to reach the target.
    pub weight: f32,
    /// How strongly [`target_rotation`](Self::target_rotation) is applied (0..=1).
    pub rotation_weight: f32,
    /// Number of parent bones that are affected. `0` = all the way to the root.
    pub chain_length: u16,
    pub flags: EffectorFlags,
}

impl Default for Effector {
    fn default() -> Self {
        Effector {
            target_position: Vec3::ZERO,
            target_rotation: Quat::IDENTITY,
            weight: 1.0,
            rotation_weight: 1.0,
            chain_length: 0,
            flags: EffectorFlags(0),
        }
    }
}

impl Effector {
    pub fn new(target_position: Vec3) -> Self {
        Effector {
            target_position,
            ..Default::default()
        }
    }
    pub fn with_chain_length(mut self, n: u16) -> Self {
        self.chain_length = n;
        self
    }
    pub fn with_weight(mut self, w: f32) -> Self {
        self.weight = w;
        self
    }
    pub fn with_flags(mut self, bits: u8) -> Self {
        self.flags = EffectorFlags(bits);
        self
    }
}

/// One bone joint.
#[derive(Clone, Debug)]
pub struct Node {
    parent: Option<NodeId>,
    children: Vec<NodeId>,
    /// Offset from the parent, expressed in the parent's local frame.
    pub position: Vec3,
    /// Rotation relative to the parent.
    pub rotation: Quat,
    pub effector: Option<Effector>,
    /// Limits the bend of the joint at this node (between the segment into it and the segment out of it).
    pub constraint: Constraint,
}

impl Node {
    pub fn parent(&self) -> Option<NodeId> {
        self.parent
    }
    pub fn children(&self) -> &[NodeId] {
        &self.children
    }
}

/// A tree of nodes. Create the root first, then add children.
#[derive(Clone, Debug, Default)]
pub struct Skeleton {
    nodes: Vec<Node>,
}

impl Skeleton {
    pub fn new() -> Self {
        Skeleton { nodes: Vec::new() }
    }

    pub fn len(&self) -> usize {
        self.nodes.len()
    }
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Creates the root node. Panics if the skeleton already has nodes.
    pub fn add_root(&mut self, position: Vec3) -> NodeId {
        assert!(self.nodes.is_empty(), "skeleton already has a root");
        self.nodes.push(Node {
            parent: None,
            children: Vec::new(),
            position,
            rotation: Quat::IDENTITY,
            effector: None,
            constraint: Constraint::None,
        });
        NodeId(0)
    }

    /// Adds a child at `offset` (in the parent's local frame).
    pub fn add_child(&mut self, parent: NodeId, offset: Vec3) -> NodeId {
        let id = NodeId(self.nodes.len() as u32);
        assert!(parent.index() < self.nodes.len(), "unknown parent");
        self.nodes.push(Node {
            parent: Some(parent),
            children: Vec::new(),
            position: offset,
            rotation: Quat::IDENTITY,
            effector: None,
            constraint: Constraint::None,
        });
        self.nodes[parent.index()].children.push(id);
        id
    }

    pub fn node(&self, id: NodeId) -> &Node {
        &self.nodes[id.index()]
    }
    pub fn node_mut(&mut self, id: NodeId) -> &mut Node {
        &mut self.nodes[id.index()]
    }
    pub fn nodes(&self) -> &[Node] {
        &self.nodes
    }

    pub fn set_effector(&mut self, id: NodeId, effector: Effector) {
        self.nodes[id.index()].effector = Some(effector);
    }
    pub fn clear_effector(&mut self, id: NodeId) {
        self.nodes[id.index()].effector = None;
    }
    pub fn set_constraint(&mut self, id: NodeId, c: Constraint) {
        self.nodes[id.index()].constraint = c;
    }

    /// Moves an existing effector's target; panics if the node has none.
    pub fn set_target(&mut self, id: NodeId, target: Vec3) {
        self.nodes[id.index()]
            .effector
            .as_mut()
            .expect("node has no effector")
            .target_position = target;
    }

    /// Global (world) positions and rotations of all nodes. Output vectors are resized.
    pub fn global_transforms(&self, pos: &mut Vec<Vec3>, rot: &mut Vec<Quat>) {
        pos.clear();
        rot.clear();
        pos.reserve(self.nodes.len());
        rot.reserve(self.nodes.len());
        for n in &self.nodes {
            match n.parent {
                None => {
                    pos.push(n.position);
                    rot.push(n.rotation);
                }
                Some(p) => {
                    let (pp, pr) = (pos[p.index()], rot[p.index()]);
                    pos.push(pp + pr.rotate(n.position));
                    rot.push(pr * n.rotation);
                }
            }
        }
    }

    /// Global position of one node.
    pub fn global_position(&self, id: NodeId) -> Vec3 {
        let mut pos = Vec::new();
        let mut rot = Vec::new();
        self.global_transforms(&mut pos, &mut rot);
        pos[id.index()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn global_transforms_follow_parents() {
        let mut s = Skeleton::new();
        let root = s.add_root(Vec3::new(1.0, 0.0, 0.0));
        let a = s.add_child(root, Vec3::new(0.0, 2.0, 0.0));
        let b = s.add_child(a, Vec3::new(0.0, 3.0, 0.0));
        s.node_mut(root).rotation = Quat::from_axis_angle(Vec3::Z, core::f32::consts::FRAC_PI_2);
        let (mut p, mut r) = (Vec::new(), Vec::new());
        s.global_transforms(&mut p, &mut r);
        // root rotated 90° about Z: local +Y becomes world -X
        assert!((p[a.index()] - Vec3::new(-1.0, 0.0, 0.0)).length() < 1e-5);
        assert!((p[b.index()] - Vec3::new(-4.0, 0.0, 0.0)).length() < 1e-5);
    }

    #[test]
    fn children_order_is_insertion_order() {
        let mut s = Skeleton::new();
        let r = s.add_root(Vec3::ZERO);
        let a = s.add_child(r, Vec3::X);
        let b = s.add_child(r, Vec3::Y);
        assert_eq!(s.node(r).children(), &[a, b]);
        assert_eq!(s.node(a).parent(), Some(r));
    }
}
