//! The snippets from the README, kept compiling and correct.

use rebirth_ik::*;

#[test]
fn quick_start() {
    let mut skel = Skeleton::new();
    let root = skel.add_root(Vec3::ZERO);
    let a = skel.add_child(root, Vec3::new(0.0, 1.0, 0.0));
    let b = skel.add_child(a, Vec3::new(0.0, 1.0, 0.0));
    let hand = skel.add_child(b, Vec3::new(0.0, 1.0, 0.0));
    skel.set_effector(hand, Effector::new(Vec3::new(1.5, 1.5, 0.0)));

    let mut solver = Fabrik::new(&skel, Config::default());
    let info = solver.solve(&mut skel);
    assert!(info.converged);
    assert!(
        skel.global_position(hand)
            .distance(Vec3::new(1.5, 1.5, 0.0))
            < 1e-2
    );
}

#[test]
fn several_effectors_constraints_and_rotations() {
    // a tiny "person": hip -> spine -> head, spine -> two arms of three bones
    let mut skel = Skeleton::new();
    let hip = skel.add_root(Vec3::ZERO);
    let spine = skel.add_child(hip, Vec3::new(0.0, 1.0, 0.0));
    let head = skel.add_child(spine, Vec3::new(0.0, 0.5, 0.0));
    let mut hand = |dir: f32| {
        let mut n = spine;
        for _ in 0..3 {
            n = skel.add_child(n, Vec3::new(dir * 0.3, 0.0, 0.0));
        }
        n
    };
    let (l_hand, r_hand) = (hand(-1.0), hand(1.0));
    skel.set_effector(head, Effector::new(Vec3::new(0.0, 1.45, 0.1)));
    skel.set_effector(
        l_hand,
        Effector::new(Vec3::new(-0.6, 1.2, 0.2)).with_chain_length(3),
    );
    skel.set_effector(
        r_hand,
        Effector::new(Vec3::new(0.6, 1.2, 0.2)).with_weight(0.5),
    );
    skel.set_constraint(
        spine,
        Constraint::Hinge {
            axis: Vec3::X,
            min: 0.0,
            max: 2.4,
        },
    );

    let cfg = Config {
        constraints: true,
        joint_rotations: true,
        ..Config::default()
    };
    let mut solver = Fabrik::new(&skel, cfg);
    let info = solver.solve(&mut skel);
    assert!(info.max_error.is_finite());
    // the spine+hip chain is the base; head and both arms are chains that start on its tip (the spine joint)
    assert_eq!(solver.chains().roots.len(), 1);
    assert_eq!(solver.chains().len(), 4);
}

#[test]
fn one_chain_no_allocation() {
    let mut points: Vec<Vec3> = (0..11).map(|i| Vec3::new(0.0, i as f32, 0.0)).collect();
    let mut lengths = [0.0; 10];
    segment_lengths(&points, &mut lengths);
    let r = solve_chain(&mut points, &lengths, Vec3::new(4.0, 5.0, 1.0), 20, 1e-3);
    assert!(r.error < 1e-2);
}

#[test]
fn two_bones_with_a_knee() {
    let (hip, knee, foot) = (
        Vec3::ZERO,
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(0.0, -2.0, 0.0),
    );
    let (new_knee, new_foot) = two_bone(
        hip,
        knee,
        foot,
        Vec3::new(0.3, -1.6, 0.0),
        Some(Vec3::new(0.0, 0.0, 5.0)),
    );
    assert!(new_foot.distance(Vec3::new(0.3, -1.6, 0.0)) < 1e-4);
    assert!(new_knee.z > 0.0, "the knee bends towards the pole");
}
