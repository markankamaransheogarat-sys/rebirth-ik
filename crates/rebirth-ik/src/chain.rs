//! Splits a bone tree into chains between effectors and branching points.
//!
//! This is the structural idea at the heart of the original `ik` library: only the nodes on a
//! path from an effector up to its `chain_length`-th ancestor take part in solving. Those paths
//! are cut into *chains* wherever two paths merge, or an effector sits in the middle of a path.
//! Chains form a tree again: a chain's *base* node is the *tip* node of its parent chain.

use alloc::vec::Vec;

use crate::skeleton::{NodeId, Skeleton};

/// A sequence of nodes ordered from the tip (effector side) to the base. Always has ≥ 2 nodes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Chain {
    pub nodes: Vec<NodeId>,
    /// Indices into [`ChainTree::chains`] of the chains whose base is this chain's tip.
    pub children: Vec<usize>,
}

impl Chain {
    pub fn tip(&self) -> NodeId {
        self.nodes[0]
    }
    pub fn base(&self) -> NodeId {
        *self.nodes.last().unwrap()
    }
    pub fn len(&self) -> usize {
        self.nodes.len()
    }
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }
}

/// All chains of a skeleton, as a forest.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ChainTree {
    pub chains: Vec<Chain>,
    /// Chains that have no parent chain.
    pub roots: Vec<usize>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mark {
    None,
    Base,
    Section,
}

impl ChainTree {
    /// Builds the chain forest for the effectors currently attached to `skel`.
    pub fn build(skel: &Skeleton) -> ChainTree {
        let n = skel.len();
        let mut tree = ChainTree::default();
        if n == 0 {
            return tree;
        }

        // 1. Mark every node that lies on a path from an effector towards the root.
        let mut marks = alloc::vec![Mark::None; n];
        for (i, node) in skel.nodes().iter().enumerate() {
            let Some(eff) = &node.effector else { continue };
            // chain_length 0 means "unlimited": the counter never reaches zero.
            let mut counter: i64 = if eff.chain_length == 0 {
                -1
            } else {
                eff.chain_length as i64
            };
            let mut cur = Some(NodeId(i as u32));
            while let Some(id) = cur {
                let marking = if counter == 0 {
                    Mark::Base
                } else {
                    Mark::Section
                };
                let slot = &mut marks[id.index()];
                if *slot == Mark::None {
                    *slot = marking;
                } else if counter != 0 {
                    // A section mark overrides a base mark.
                    *slot = marking;
                }
                let stop = counter == 0;
                counter -= 1;
                if stop {
                    break;
                }
                cur = skel.node(id).parent();
            }
        }

        // 2. Walk the tree depth-first and cut the marked nodes into chains.
        struct Frame {
            node: NodeId,
            chain: Option<usize>,
            base: NodeId,
        }
        let root = NodeId(0);
        let mut stack = alloc::vec![Frame {
            node: root,
            chain: None,
            base: root
        }];
        while let Some(Frame { node, chain, base }) = stack.pop() {
            // `node_base` is the base the chain ending at this node would use; `child_base` is what we
            // hand to the children. `current_chain` is where a new chain would be attached.
            let mut node_base = base;
            let mut child_base = base;
            let mut child_chain = chain;
            let mut current_chain = chain;

            let marking = core::mem::replace(&mut marks[node.index()], Mark::None);
            let mut section_logic = false;
            match marking {
                Mark::Base => {
                    child_base = node;
                    current_chain = None;
                    child_chain = None;
                }
                Mark::None => {
                    node_base = node;
                    child_base = node;
                    section_logic = true;
                }
                Mark::Section => {
                    section_logic = true;
                }
            }

            if section_logic {
                let marked_children = skel
                    .node(node)
                    .children()
                    .iter()
                    .filter(|c| marks[c.index()] == Mark::Section)
                    .take(2)
                    .count();
                let has_effector = skel.node(node).effector.is_some();
                if (marked_children == 2 || has_effector) && node != node_base {
                    let mut nodes = Vec::new();
                    let mut cur = node;
                    while cur != node_base {
                        nodes.push(cur);
                        cur = skel.node(cur).parent().expect("base must be an ancestor");
                    }
                    nodes.push(node_base);
                    let idx = tree.chains.len();
                    tree.chains.push(Chain {
                        nodes,
                        children: Vec::new(),
                    });
                    match current_chain {
                        None => tree.roots.push(idx),
                        Some(parent) => tree.chains[parent].children.push(idx),
                    }
                    child_chain = Some(idx);
                    child_base = node;
                }
            }
            for &c in skel.node(node).children().iter().rev() {
                stack.push(Frame {
                    node: c,
                    chain: child_chain,
                    base: child_base,
                });
            }
        }
        tree
    }

    /// Number of chains.
    pub fn len(&self) -> usize {
        self.chains.len()
    }
    pub fn is_empty(&self) -> bool {
        self.chains.is_empty()
    }
}
