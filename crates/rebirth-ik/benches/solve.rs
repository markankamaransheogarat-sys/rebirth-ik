use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use rebirth_ik::*;

fn chain_points(n: usize) -> Vec<Vec3> {
    (0..n).map(|i| Vec3::new(0.0, i as f32, 0.0)).collect()
}

/// Fixed work: always 20 sweeps (tolerance 0 never "converges early").
fn bench_single_chain(c: &mut Criterion) {
    let mut g = c.benchmark_group("single_chain_20_sweeps");
    for &bones in &[5usize, 10, 20, 50, 200] {
        let base = chain_points(bones + 1);
        let mut lengths = vec![0.0; bones];
        segment_lengths(&base, &mut lengths);
        let target = Vec3::new(bones as f32 * 0.4, bones as f32 * 0.5, bones as f32 * 0.2);
        g.bench_with_input(BenchmarkId::from_parameter(bones), &bones, |b, _| {
            let mut pts = base.clone();
            b.iter(|| {
                pts.copy_from_slice(&base);
                black_box(solve_chain(&mut pts, &lengths, black_box(target), 20, 0.0));
            });
        });
    }
    g.finish();
}

/// A humanoid-ish tree: spine, two arms, two legs = 5 effectors sharing sub-bases.
fn humanoid() -> Skeleton {
    let mut s = Skeleton::new();
    let hip = s.add_root(Vec3::ZERO);
    let mut spine = hip;
    for _ in 0..3 {
        spine = s.add_child(spine, Vec3::new(0.0, 0.3, 0.0));
    }
    let neck = s.add_child(spine, Vec3::new(0.0, 0.2, 0.0));
    let head = s.add_child(neck, Vec3::new(0.0, 0.2, 0.0));
    let limb = |s: &mut Skeleton, from: NodeId, dir: Vec3| {
        let mut n = from;
        for _ in 0..3 {
            n = s.add_child(n, dir * 0.35);
        }
        n
    };
    let la = limb(&mut s, spine, Vec3::new(-1.0, 0.0, 0.0));
    let ra = limb(&mut s, spine, Vec3::new(1.0, 0.0, 0.0));
    let ll = limb(&mut s, hip, Vec3::new(-0.2, -1.0, 0.0));
    let rl = limb(&mut s, hip, Vec3::new(0.2, -1.0, 0.0));
    for (n, t) in [
        (head, Vec3::new(0.1, 1.3, 0.2)),
        (la, Vec3::new(-0.9, 0.7, 0.4)),
        (ra, Vec3::new(0.9, 0.9, 0.2)),
        (ll, Vec3::new(-0.3, -0.9, 0.2)),
        (rl, Vec3::new(0.3, -0.9, -0.1)),
    ] {
        s.set_effector(n, Effector::new(t));
    }
    s
}

fn bench_tree(c: &mut Criterion) {
    let mut g = c.benchmark_group("tree_humanoid_5_effectors_20_sweeps");
    let template = humanoid();
    let cfg = Config {
        max_iterations: 20,
        tolerance: 0.0,
        early_exit: false,
        ..Config::default()
    };
    g.bench_function("positions_only", |b| {
        let mut sk = template.clone();
        let mut solver = Fabrik::new(&sk, cfg);
        b.iter(|| {
            sk = template.clone();
            black_box(solver.solve(&mut sk));
        });
    });
    g.bench_function("with_joint_rotations", |b| {
        let mut sk = template.clone();
        let mut solver = Fabrik::new(
            &sk,
            Config {
                joint_rotations: true,
                ..cfg
            },
        );
        b.iter(|| {
            sk = template.clone();
            black_box(solver.solve(&mut sk));
        });
    });
    g.finish();
}

fn bench_two_bone(c: &mut Criterion) {
    c.bench_function("two_bone_analytic", |b| {
        let (base, mid, tip) = (
            Vec3::ZERO,
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
        );
        b.iter(|| {
            black_box(two_bone(
                base,
                mid,
                tip,
                black_box(Vec3::new(0.7, 1.1, 0.3)),
                Some(Vec3::new(0.0, 0.0, 5.0)),
            ))
        });
    });
}

fn crowd(n: usize) -> Vec<ChainBuf> {
    (0..n)
        .map(|i| {
            let f = i as f32;
            ChainBuf::new(
                chain_points(11),
                Vec3::new(
                    (f * 0.37).sin() * 6.0,
                    3.0 + (f * 0.11).cos() * 3.0,
                    (f * 0.53).sin() * 6.0,
                ),
            )
        })
        .collect()
}

fn bench_crowd(c: &mut Criterion) {
    let mut g = c.benchmark_group("crowd_10bone_chains_20_sweeps");
    for &n in &[1_000usize, 10_000, 100_000] {
        g.throughput(Throughput::Elements(n as u64));
        let template = crowd(n);
        g.bench_with_input(BenchmarkId::new("serial", n), &n, |b, _| {
            let mut chains = template.clone();
            b.iter(|| {
                for (c, t) in chains.iter_mut().zip(&template) {
                    c.positions.copy_from_slice(&t.positions);
                }
                solve_chains(&mut chains, 20, 0.0);
            });
        });
        #[cfg(feature = "rayon")]
        g.bench_with_input(BenchmarkId::new("rayon", n), &n, |b, _| {
            let mut chains = template.clone();
            b.iter(|| {
                for (c, t) in chains.iter_mut().zip(&template) {
                    c.positions.copy_from_slice(&t.positions);
                }
                solve_chains_par(&mut chains, 20, 0.0);
            });
        });
    }
    g.finish();
}

criterion_group!(
    benches,
    bench_single_chain,
    bench_tree,
    bench_two_bone,
    bench_crowd
);
criterion_main!(benches);
