use rebirth_ik::*;
use std::f32::consts::FRAC_PI_6;

/// Straight vertical chain of `bones` unit bones. Returns the skeleton and the tip id.
fn arm(bones: usize) -> (Skeleton, NodeId) {
    let mut s = Skeleton::new();
    let mut n = s.add_root(Vec3::ZERO);
    for _ in 0..bones {
        n = s.add_child(n, Vec3::new(0.0, 1.0, 0.0));
    }
    (s, n)
}

fn globals(s: &Skeleton) -> Vec<Vec3> {
    let (mut p, mut r) = (Vec::new(), Vec::new());
    s.global_transforms(&mut p, &mut r);
    p
}

fn bone_lengths_preserved(s: &Skeleton, expect: f32) -> bool {
    let p = globals(s);
    s.nodes().iter().enumerate().all(|(i, n)| match n.parent() {
        Some(par) => (p[i].distance(p[par.index()]) - expect).abs() < 1e-3,
        None => true,
    })
}

#[test]
fn chain_reaches_target() {
    let (mut s, tip) = arm(10);
    let target = Vec3::new(4.0, 5.0, 3.0);
    s.set_effector(tip, Effector::new(target));
    let mut solver = Fabrik::new(
        &s,
        Config {
            max_iterations: 100,
            tolerance: 1e-4,
            ..Config::default()
        },
    );
    let info = solver.solve(&mut s);
    assert!(info.converged, "{info:?}");
    assert!(globals(&s)[tip.index()].distance(target) < 1e-3);
    assert!(bone_lengths_preserved(&s, 1.0));
    assert_eq!(globals(&s)[0], Vec3::ZERO, "root stays");
}

#[test]
fn unreachable_target_stretches_straight() {
    let (mut s, tip) = arm(4);
    s.set_effector(tip, Effector::new(Vec3::new(50.0, 0.0, 0.0)));
    let mut solver = Fabrik::new(
        &s,
        Config {
            max_iterations: 200,
            ..Config::default()
        },
    );
    let info = solver.solve(&mut s);
    assert!(!info.converged);
    let p = globals(&s)[tip.index()];
    assert!(p.distance(Vec3::new(4.0, 0.0, 0.0)) < 0.05, "tip at {p:?}");
    assert!(bone_lengths_preserved(&s, 1.0));
}

#[test]
fn early_exit_stops_when_close_enough() {
    let (mut s, tip) = arm(3);
    let here = globals(&s)[tip.index()];
    s.set_effector(tip, Effector::new(here));
    let mut solver = Fabrik::new(&s, Config::default());
    assert_eq!(solver.solve(&mut s).iterations, 0);

    solver.config.early_exit = false;
    assert_eq!(solver.solve(&mut s).iterations, 20);
}

#[test]
fn solving_twice_is_stable() {
    let (mut s, tip) = arm(6);
    s.set_effector(tip, Effector::new(Vec3::new(2.0, 3.0, 1.0)));
    let mut solver = Fabrik::new(
        &s,
        Config {
            max_iterations: 100,
            tolerance: 1e-5,
            ..Config::default()
        },
    );
    solver.solve(&mut s);
    let a = globals(&s);
    let info = solver.solve(&mut s);
    let b = globals(&s);
    assert_eq!(info.iterations, 0);
    for (x, y) in a.iter().zip(&b) {
        assert!(x.distance(*y) < 1e-4);
    }
}

/// A spine with two arms branching off its top (like shoulders).
fn spine_with_two_arms() -> (Skeleton, NodeId, NodeId) {
    let mut s = Skeleton::new();
    let hip = s.add_root(Vec3::ZERO);
    // A slight S-curve: a perfectly collinear spine is a classic FABRIK singularity (it cannot fold),
    // and real spines are never perfectly straight.
    let waist = s.add_child(hip, Vec3::new(0.15, 1.0, 0.0));
    let chest = s.add_child(waist, Vec3::new(-0.15, 1.0, 0.0));
    let (mut l, mut r) = (chest, chest);
    for _ in 0..3 {
        l = s.add_child(l, Vec3::new(-1.0, 0.0, 0.0));
        r = s.add_child(r, Vec3::new(1.0, 0.0, 0.0));
    }
    (s, l, r)
}

#[test]
fn two_effectors_sharing_a_spine_are_both_reached() {
    let (mut s, l, r) = spine_with_two_arms();
    let (tl, tr) = (Vec3::new(-2.0, 4.0, 1.0), Vec3::new(2.5, 3.0, -1.0));
    s.set_effector(l, Effector::new(tl).with_chain_length(3));
    s.set_effector(r, Effector::new(tr).with_chain_length(3));
    let mut solver = Fabrik::new(
        &s,
        Config {
            max_iterations: 200,
            tolerance: 1e-3,
            ..Config::default()
        },
    );
    let before_hip = globals(&s)[0];
    let info = solver.solve(&mut s);
    let p = globals(&s);
    // The arms are anchored at the (immovable) chest, so the targets are both within the arm length 3 of the chest.
    assert!(
        p[l.index()].distance(tl) < 0.05,
        "{info:?} left tip {:?}",
        p[l.index()]
    );
    assert!(
        p[r.index()].distance(tr) < 0.05,
        "{info:?} right tip {:?}",
        p[r.index()]
    );
    assert_eq!(p[0], before_hip);
}

#[test]
fn full_height_chain_moves_the_shared_spine() {
    // Unlimited chains reach down to the hip, so the shared spine has to compromise between both arms.
    let (mut s, l, r) = spine_with_two_arms();
    // Both targets are below the rest pose: each arm alone is too short from the chest at (0,2),
    // so the spine has to fold down to bring the shoulders closer.
    let (tl, tr) = (Vec3::new(-2.5, -1.0, 0.0), Vec3::new(2.5, -1.0, 0.0));
    s.set_effector(l, Effector::new(tl));
    s.set_effector(r, Effector::new(tr));
    let mut solver = Fabrik::new(
        &s,
        Config {
            max_iterations: 500,
            tolerance: 1e-3,
            ..Config::default()
        },
    );
    let info = solver.solve(&mut s);
    let p = globals(&s);
    assert!(info.max_error < 0.05, "{info:?}");
    assert!(p[l.index()].distance(tl) < 0.05 && p[r.index()].distance(tr) < 0.05);
    assert_eq!(p[0], Vec3::ZERO, "the hip is the base and stays");
    assert!(bone_lengths_preserved_any(&s));
    assert!(p
        .iter()
        .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()));
}

fn bone_lengths_preserved_any(s: &Skeleton) -> bool {
    // lengths are local offsets: compare global distance with the local offset length
    let p = globals(s);
    s.nodes().iter().enumerate().all(|(i, n)| match n.parent() {
        Some(par) => (p[i].distance(p[par.index()]) - n.position.length()).abs() < 1e-3,
        None => true,
    })
}

#[test]
fn effector_weight_blends_the_target() {
    let (mut s, tip) = arm(5);
    let start = globals(&s)[tip.index()];
    let target = Vec3::new(2.0, 3.0, 0.0);
    s.set_effector(tip, Effector::new(target).with_weight(0.5));
    let mut solver = Fabrik::new(
        &s,
        Config {
            max_iterations: 200,
            tolerance: 1e-4,
            ..Config::default()
        },
    );
    solver.solve(&mut s);
    let tip_now = globals(&s)[tip.index()];
    assert!(
        tip_now.distance(start.lerp(target, 0.5)) < 1e-2,
        "tip {tip_now:?}"
    );
}

#[test]
fn zero_weight_leaves_the_pose_alone() {
    let (mut s, tip) = arm(5);
    let before = globals(&s);
    s.set_effector(
        tip,
        Effector::new(Vec3::new(9.0, 9.0, 9.0)).with_weight(0.0),
    );
    let mut solver = Fabrik::new(&s, Config::default());
    solver.solve(&mut s);
    for (a, b) in before.iter().zip(globals(&s)) {
        assert!(a.distance(b) < 1e-4);
    }
}

#[test]
fn nlerp_weight_keeps_the_chain_length_consistent() {
    let (mut s, tip) = arm(5);
    s.set_effector(
        tip,
        Effector::new(Vec3::new(5.0, 0.0, 0.0))
            .with_weight(0.5)
            .with_flags(EffectorFlags::WEIGHT_NLERP),
    );
    let mut solver = Fabrik::new(
        &s,
        Config {
            max_iterations: 100,
            ..Config::default()
        },
    );
    solver.solve(&mut s);
    // Rest distance and target distance from the root are both 5, so the nlerp'd tip stays on a
    // radius of 5 (a fully stretched chain converges slowly, hence the loose tolerance) and has
    // swung halfway between +Y and +X.
    let p = globals(&s)[tip.index()];
    assert!((p.length() - 5.0).abs() < 0.05, "distance {}", p.length());
    assert!(p.x > 2.5 && p.y > 2.5, "tip at {p:?}");
}

#[test]
fn joint_rotations_turn_the_bones() {
    let (mut s, tip) = arm(3);
    let target = Vec3::new(2.0, 1.5, 0.5);
    s.set_effector(tip, Effector::new(target));
    let mut solver = Fabrik::new(
        &s,
        Config {
            max_iterations: 100,
            tolerance: 1e-4,
            joint_rotations: true,
            ..Config::default()
        },
    );
    solver.solve(&mut s);
    let p = globals(&s);
    // Every joint's rotation maps the rest-pose bone direction (+Y) onto the solved segment direction.
    // (Rotations are global here because the chain starts at the root with identity rotation.)
    let (mut gp, mut gr) = (Vec::new(), Vec::new());
    s.global_transforms(&mut gp, &mut gr);
    for i in 0..3 {
        let solved = (p[i + 1] - p[i]).normalized();
        let rotated = gr[i].rotate(Vec3::Y);
        assert!(
            rotated.distance(solved) < 1e-2,
            "joint {i}: rotated {rotated:?} vs segment {solved:?}"
        );
    }
    // Local offsets keep their original shape: pure rotation, nothing stretched.
    for i in 1..s.len() {
        let off = s.node(NodeId(i as u32)).position;
        assert!(off.distance(Vec3::Y) < 1e-2, "node {i} offset {off:?}");
    }
    assert!(p[tip.index()].distance(target) < 1e-2);
}

#[test]
fn cone_constraints_limit_bending() {
    let (mut s, tip) = arm(4);
    for i in 1..4 {
        s.set_constraint(
            NodeId(i),
            Constraint::Cone {
                max_angle: FRAC_PI_6,
            },
        );
    }
    s.set_effector(tip, Effector::new(Vec3::new(3.0, 0.5, 0.0)));
    let mut solver = Fabrik::new(
        &s,
        Config {
            max_iterations: 100,
            constraints: true,
            ..Config::default()
        },
    );
    solver.solve(&mut s);
    let p = globals(&s);
    for i in 1..3 {
        let a = (p[i] - p[i - 1]).normalized();
        let b = (p[i + 1] - p[i]).normalized();
        let angle = a.dot(b).clamp(-1.0, 1.0).acos();
        assert!(angle <= FRAC_PI_6 + 1e-3, "joint {i} bends {angle} rad");
    }
    assert!(bone_lengths_preserved(&s, 1.0));
}

#[test]
fn stiff_constraint_keeps_the_chain_straight() {
    let (mut s, tip) = arm(3);
    for i in 1..3 {
        s.set_constraint(NodeId(i), Constraint::Stiff);
    }
    s.set_effector(tip, Effector::new(Vec3::new(2.0, 1.0, 0.0)));
    let mut solver = Fabrik::new(
        &s,
        Config {
            max_iterations: 50,
            constraints: true,
            ..Config::default()
        },
    );
    solver.solve(&mut s);
    let p = globals(&s);
    let d0 = (p[1] - p[0]).normalized();
    for i in 2..4 {
        assert!(((p[i] - p[i - 1]).normalized()).distance(d0) < 1e-3);
    }
}

#[test]
fn rebuild_after_changing_effectors() {
    let (mut s, tip) = arm(4);
    let mut solver = Fabrik::new(&s, Config::default());
    assert!(solver.chains().is_empty());
    s.set_effector(tip, Effector::new(Vec3::new(1.0, 2.0, 0.0)));
    solver.rebuild(&s);
    assert_eq!(solver.chains().len(), 1);
    assert!(solver.solve(&mut s).converged);
}

#[test]
fn nodes_below_the_effector_follow_rigidly() {
    // 0-1-2(effector)-3: node 3 is not part of any chain and must ride along with node 2
    let (mut s, n3) = arm(3);
    let n2 = s.node(n3).parent().unwrap();
    let target = Vec3::new(1.5, 0.5, 0.0); // within the two-bone reach (2.0) of the root
    s.set_effector(n2, Effector::new(target));
    let mut solver = Fabrik::new(
        &s,
        Config {
            max_iterations: 100,
            tolerance: 1e-4,
            joint_rotations: true,
            ..Config::default()
        },
    );
    solver.solve(&mut s);
    let p = globals(&s);
    assert!(p[n2.index()].distance(target) < 1e-2);
    assert!(
        (p[n3.index()].distance(p[n2.index()]) - 1.0).abs() < 1e-3,
        "bone below the effector keeps its length"
    );
}
