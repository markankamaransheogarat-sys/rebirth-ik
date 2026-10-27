//! Joint limits used by the tree solver.

use crate::math::{fm, Quat, Vec3};

/// Limits how far a joint may bend.
///
/// A constraint attached to node `N` restricts the angle between the segment *into* `N`
/// (parent → `N`) and the segment *out of* `N` (`N` → child). Constraints are enforced in the
/// backward FABRIK pass, so the solved pose always respects them (the effector may then
/// stop short of its target if the limits make it unreachable).
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum Constraint {
    /// No limit.
    #[default]
    None,
    /// The outgoing segment continues straight along the incoming one.
    Stiff,
    /// Bend around a fixed world-space `axis` only, between `min` and `max` radians
    /// relative to the incoming segment (0 = straight, positive = counter-clockwise about `axis`).
    Hinge { axis: Vec3, min: f32, max: f32 },
    /// The outgoing segment stays within `max_angle` radians of the incoming direction.
    Cone { max_angle: f32 },
}

impl Constraint {
    /// Projects `dir` (unit) into the allowed region relative to `prev` (unit, the incoming direction).
    pub(crate) fn limit(&self, prev: Vec3, dir: Vec3) -> Vec3 {
        match *self {
            Constraint::None => dir,
            Constraint::Stiff => prev,
            Constraint::Cone { max_angle } => {
                let angle = fm::acos(prev.dot(dir).clamp(-1.0, 1.0));
                if angle <= max_angle {
                    return dir;
                }
                let mut axis = prev.cross(dir);
                if axis.length_squared() < 1e-12 {
                    axis = prev.any_perpendicular();
                }
                Quat::from_axis_angle(axis, max_angle).rotate(prev)
            }
            Constraint::Hinge { axis, min, max } => {
                let a = axis.normalized();
                if a == Vec3::ZERO {
                    return dir;
                }
                let p = prev - a * prev.dot(a);
                let d = dir - a * dir.dot(a);
                if p.length_squared() < 1e-12 || d.length_squared() < 1e-12 {
                    return prev;
                }
                let theta = fm::atan2(a.dot(p.cross(d)), p.dot(d));
                let clamped = theta.clamp(min, max);
                Quat::from_axis_angle(a, clamped).rotate(prev)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::{FRAC_PI_2, FRAC_PI_4};

    fn angle(a: Vec3, b: Vec3) -> f32 {
        fm::acos(a.normalized().dot(b.normalized()).clamp(-1.0, 1.0))
    }

    #[test]
    fn cone_clamps_to_limit() {
        let c = Constraint::Cone {
            max_angle: FRAC_PI_4,
        };
        let out = c.limit(Vec3::X, Vec3::Y);
        assert!((angle(Vec3::X, out) - FRAC_PI_4).abs() < 1e-4);
        // inside the cone: untouched
        let inside = Vec3::new(1.0, 0.1, 0.0).normalized();
        assert_eq!(c.limit(Vec3::X, inside), inside);
        // anti-parallel input still gives a valid direction at the limit
        let back = c.limit(Vec3::X, -Vec3::X);
        assert!((angle(Vec3::X, back) - FRAC_PI_4).abs() < 1e-4);
    }

    #[test]
    fn stiff_forces_straight() {
        assert_eq!(Constraint::Stiff.limit(Vec3::X, Vec3::Y), Vec3::X);
    }

    #[test]
    fn hinge_limits_signed_angle() {
        let c = Constraint::Hinge {
            axis: Vec3::Z,
            min: 0.0,
            max: FRAC_PI_2,
        };
        // +Y is +90° from +X about +Z: allowed
        assert!((c.limit(Vec3::X, Vec3::Y) - Vec3::Y).length() < 1e-4);
        // -Y is -90°: below min, clamped to 0 (straight)
        assert!((c.limit(Vec3::X, -Vec3::Y) - Vec3::X).length() < 1e-4);
        // 135° is above max: clamped to 90°
        let d = Vec3::new(-1.0, 1.0, 0.0).normalized();
        assert!((c.limit(Vec3::X, d) - Vec3::Y).length() < 1e-4);
    }
}
