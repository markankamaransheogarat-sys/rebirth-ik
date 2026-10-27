//! Closed-form solvers for chains of one and two bones. Exact, branch-light and allocation-free.

use crate::math::{fm, Quat, Vec3};

/// One bone: points the bone from `base` at `target`, keeping its length.
/// Returns the new tip position.
pub fn one_bone(base: Vec3, tip: Vec3, target: Vec3) -> Vec3 {
    let len = base.distance(tip);
    let d = (target - base).normalized();
    if d == Vec3::ZERO {
        tip
    } else {
        base + d * len
    }
}

/// Two bones (e.g. upper arm + forearm, thigh + shin) solved with the law of cosines.
///
/// `base`, `mid`, `tip` are the current joint positions (only used for the bone lengths and, when
/// no `pole` is given, to keep the bend plane stable). `pole` is an optional point the middle
/// joint should bend towards (the knee/elbow target). Returns the new `(mid, tip)`.
///
/// If the target is out of reach, both bones point straight at it; if it is closer than the
/// bones can fold, the chain folds as far as possible.
pub fn two_bone(
    base: Vec3,
    mid: Vec3,
    tip: Vec3,
    target: Vec3,
    pole: Option<Vec3>,
) -> (Vec3, Vec3) {
    let upper = base.distance(mid); // base → mid
    let lower = mid.distance(tip); // mid → tip
    let to_target = target - base;
    let dist = to_target.length();
    if dist < 1e-9 || upper < 1e-9 || lower < 1e-9 {
        return (mid, tip);
    }
    let dir = to_target * (1.0 / dist);

    // Unreachable: stretch straight.
    if dist >= upper + lower {
        let m = base + dir * upper;
        return (m, m + dir * lower);
    }
    // Too close: fold as far as the shorter bone allows.
    let min_reach = (upper - lower).abs().max(1e-6);
    let d = dist.max(min_reach);

    // Angle at the base between the base→target line and the upper bone.
    let cos_a = ((upper * upper + d * d - lower * lower) / (2.0 * upper * d)).clamp(-1.0, 1.0);
    let sin_a = fm::sqrt((1.0 - cos_a * cos_a).max(0.0));

    // Bend direction: perpendicular to `dir`, towards the pole (or the current elbow position).
    let hint = match pole {
        Some(p) => p - base,
        None => mid - base,
    };
    let mut side = hint - dir * hint.dot(dir);
    if side.length_squared() < 1e-12 {
        side = dir.any_perpendicular();
    }
    let side = side.normalized();

    let m = base + dir * (upper * cos_a) + side * (upper * sin_a);
    let t = if dist >= min_reach {
        target
    } else {
        m + (target - m).normalized() * lower
    };
    (m, t)
}

/// Rotation that turns the rest-pose segment direction `from` into the solved `to`.
/// Convenience for applying analytic results to bone rotations.
pub fn segment_rotation(from: Vec3, to: Vec3) -> Quat {
    Quat::from_to(from, to)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_bone_keeps_length() {
        let tip = one_bone(
            Vec3::ZERO,
            Vec3::new(0.0, 2.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
        );
        assert!((tip - Vec3::new(2.0, 0.0, 0.0)).length() < 1e-5);
    }

    #[test]
    fn two_bone_reaches_target_and_keeps_lengths() {
        let (base, mid, tip) = (
            Vec3::ZERO,
            Vec3::new(0.0, 2.0, 0.0),
            Vec3::new(0.0, 5.0, 0.0),
        );
        let target = Vec3::new(2.0, 2.0, 1.0);
        let (m, t) = two_bone(base, mid, tip, target, None);
        assert!((t - target).length() < 1e-4);
        assert!((base.distance(m) - 2.0).abs() < 1e-4);
        assert!((m.distance(t) - 3.0).abs() < 1e-4);
    }

    #[test]
    fn two_bone_unreachable_is_straight() {
        let (m, t) = two_bone(
            Vec3::ZERO,
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
            Vec3::new(0.0, 0.0, 9.0),
            None,
        );
        assert!((m - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-5);
        assert!((t - Vec3::new(0.0, 0.0, 2.0)).length() < 1e-5);
    }

    #[test]
    fn two_bone_pole_picks_bend_side() {
        let (base, mid, tip) = (
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
        );
        let target = Vec3::new(1.2, 0.0, 0.0); // forces a bend
        let (up, _) = two_bone(base, mid, tip, target, Some(Vec3::new(0.0, 5.0, 0.0)));
        let (down, _) = two_bone(base, mid, tip, target, Some(Vec3::new(0.0, -5.0, 0.0)));
        assert!(up.y > 0.1 && down.y < -0.1);
        assert!((up.y + down.y).abs() < 1e-4);
    }

    #[test]
    fn two_bone_collinear_input_does_not_nan() {
        // the original C solver divides by a zero cross product here
        let (m, t) = two_bone(
            Vec3::ZERO,
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            None,
        );
        assert!(m.x.is_finite() && m.y.is_finite() && m.z.is_finite());
        assert!(t.x.is_finite() && t.y.is_finite() && t.z.is_finite());
    }
}
