//! Chain decomposition tests. The scenarios come from the (commented-out) test suite of the
//! original TheComet/ik `test_FABRIK.cpp`; there they were never enabled, here they run.

use rebirth_ik::*;

/// Builds a skeleton where `parents[i]` is the parent of node `i + 1` (node 0 is the root).
/// Every bone is 1 unit long, pointing up.
fn tree(parents: &[u32]) -> Skeleton {
    let mut s = Skeleton::new();
    s.add_root(Vec3::ZERO);
    for &p in parents {
        s.add_child(NodeId(p), Vec3::new(0.0, 1.0, 0.0));
    }
    s
}

fn ids(c: &Chain) -> Vec<u32> {
    c.nodes.iter().map(|n| n.0).collect()
}

#[test]
fn weird_tree() {
    //            0-1-2-3-4
    //                 \
    //                  5-6-7
    //                     \
    //                      8-9-10-11-12
    let mut s = tree(&[0, 1, 2, 3, 2, 5, 6, 6, 8, 9, 10, 11]);
    s.set_effector(NodeId(4), Effector::default().with_chain_length(1));
    s.set_effector(NodeId(7), Effector::default().with_chain_length(4));
    s.set_effector(NodeId(11), Effector::default().with_chain_length(7));

    let t = ChainTree::build(&s);
    assert_eq!(t.roots.len(), 2, "two separate chain trees");

    let c1 = &t.chains[t.roots[0]];
    assert_eq!(ids(c1), [4, 3]);
    assert!(c1.children.is_empty());

    let c2 = &t.chains[t.roots[1]];
    assert_eq!(ids(c2), [6, 5, 2, 1]);
    assert_eq!(c2.children.len(), 2);

    let c3 = &t.chains[c2.children[0]];
    assert_eq!(ids(c3), [7, 6]);
    let c4 = &t.chains[c2.children[1]];
    assert_eq!(ids(c4), [11, 10, 9, 8, 6]);
    assert!(c3.children.is_empty() && c4.children.is_empty());
}

#[test]
fn just_one_node_has_no_chains() {
    let mut s = tree(&[]);
    s.set_effector(NodeId(0), Effector::default());
    assert!(ChainTree::build(&s).is_empty());
}

#[test]
fn two_arms_meet_at_same_node() {
    // 0-1-2-3-4 and 2-5-6
    let mut s = tree(&[0, 1, 2, 3, 2, 5]);
    s.set_effector(NodeId(4), Effector::default().with_chain_length(2));
    s.set_effector(NodeId(6), Effector::default().with_chain_length(2));
    let t = ChainTree::build(&s);
    assert_eq!(t.roots.len(), 2);
    assert_eq!(ids(&t.chains[t.roots[0]]), [4, 3, 2]);
    assert_eq!(ids(&t.chains[t.roots[1]]), [6, 5, 2]);
    assert!(t.chains.iter().all(|c| c.children.is_empty()));
}

#[test]
fn two_separate_arms() {
    // 0-1-2-3 and 1-4-5
    let mut s = tree(&[0, 1, 2, 1, 4]);
    s.set_effector(NodeId(3), Effector::default().with_chain_length(1));
    s.set_effector(NodeId(5), Effector::default().with_chain_length(1));
    let t = ChainTree::build(&s);
    assert_eq!(t.roots.len(), 2);
    assert_eq!(ids(&t.chains[t.roots[0]]), [3, 2]);
    assert_eq!(ids(&t.chains[t.roots[1]]), [5, 4]);
}

#[test]
fn effector_in_the_middle_of_a_chain_splits_it() {
    // 0-1-2-3-4-5-6, effectors at 3 and 6 (unlimited)
    let mut s = tree(&[0, 1, 2, 3, 4, 5]);
    s.set_effector(NodeId(3), Effector::default());
    s.set_effector(NodeId(6), Effector::default());
    let t = ChainTree::build(&s);
    assert_eq!(t.roots.len(), 1);
    let bottom = &t.chains[t.roots[0]];
    assert_eq!(ids(bottom), [3, 2, 1, 0]);
    assert_eq!(bottom.children.len(), 1);
    let top = &t.chains[bottom.children[0]];
    assert_eq!(ids(top), [6, 5, 4, 3]);
    assert!(top.children.is_empty());
}

#[test]
fn zero_chain_length_reaches_the_root() {
    let mut s = tree(&[0, 1, 2, 3]);
    s.set_effector(NodeId(4), Effector::default());
    let t = ChainTree::build(&s);
    assert_eq!(ids(&t.chains[0]), [4, 3, 2, 1, 0]);
}

#[test]
fn chain_length_limits_the_bones() {
    let mut s = tree(&[0, 1, 2, 3]);
    s.set_effector(NodeId(4), Effector::default().with_chain_length(2));
    let t = ChainTree::build(&s);
    assert_eq!(ids(&t.chains[0]), [4, 3, 2]);
}

fn build_long(s_parents: &mut Vec<u32>, parent: u32, depth: u32) -> Vec<(u32, u32)> {
    // each level: two arms of three nodes; leaves at max depth get an effector on both arm ends
    let mut effs = Vec::new();
    let base = s_parents.len() as u32 + 1;
    let ids: Vec<u32> = (0..6).map(|i| base + i).collect();
    s_parents.extend_from_slice(&[parent, ids[0], ids[1], parent, ids[3], ids[4]]);
    if depth < 3 {
        effs.extend(build_long(s_parents, ids[2], depth + 1));
        effs.extend(build_long(s_parents, ids[5], depth + 1));
    } else {
        effs.push((ids[2], 0));
        effs.push((ids[5], 0));
    }
    effs
}

#[test]
fn binary_tree_with_long_chains_builds_consistent_chains() {
    let mut parents = Vec::new();
    let effs = build_long(&mut parents, 0, 0);
    let mut s = tree(&parents);
    for (n, _) in &effs {
        s.set_effector(NodeId(*n), Effector::default());
    }
    let t = ChainTree::build(&s);

    // every effector is the tip of exactly one chain
    let tips: Vec<u32> = t.chains.iter().map(|c| c.tip().0).collect();
    for (n, _) in &effs {
        assert_eq!(tips.iter().filter(|&&x| x == *n).count(), 1, "effector {n}");
    }
    // every chain has >= 2 nodes and a child chain's base is its parent's tip
    for c in &t.chains {
        assert!(c.len() >= 2);
        for &ch in &c.children {
            assert_eq!(t.chains[ch].base(), c.tip());
        }
    }
    // consecutive nodes of a chain are parent/child in the skeleton
    for c in &t.chains {
        for w in c.nodes.windows(2) {
            assert_eq!(s.node(w[0]).parent(), Some(w[1]));
        }
    }
}
