//! Minimal `f32` vector and quaternion math. No allocation, `no_std`-friendly.

use core::ops::{Add, AddAssign, Div, Mul, MulAssign, Neg, Sub, SubAssign};

/// Float functions that work with or without `std`.
pub(crate) mod fm {
    #[cfg(feature = "std")]
    #[inline(always)]
    pub fn sqrt(x: f32) -> f32 {
        x.sqrt()
    }
    #[cfg(feature = "std")]
    #[inline(always)]
    pub fn sin(x: f32) -> f32 {
        x.sin()
    }
    #[cfg(feature = "std")]
    #[inline(always)]
    pub fn cos(x: f32) -> f32 {
        x.cos()
    }
    #[cfg(feature = "std")]
    #[inline(always)]
    pub fn acos(x: f32) -> f32 {
        x.acos()
    }
    #[cfg(feature = "std")]
    #[inline(always)]
    pub fn atan2(y: f32, x: f32) -> f32 {
        y.atan2(x)
    }

    #[cfg(not(feature = "std"))]
    #[inline(always)]
    pub fn sqrt(x: f32) -> f32 {
        libm::sqrtf(x)
    }
    #[cfg(not(feature = "std"))]
    #[inline(always)]
    pub fn sin(x: f32) -> f32 {
        libm::sinf(x)
    }
    #[cfg(not(feature = "std"))]
    #[inline(always)]
    pub fn cos(x: f32) -> f32 {
        libm::cosf(x)
    }
    #[cfg(not(feature = "std"))]
    #[inline(always)]
    pub fn acos(x: f32) -> f32 {
        libm::acosf(x)
    }
    #[cfg(not(feature = "std"))]
    #[inline(always)]
    pub fn atan2(y: f32, x: f32) -> f32 {
        libm::atan2f(y, x)
    }
}

/// A 3D vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    pub const X: Vec3 = Vec3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };
    pub const Y: Vec3 = Vec3 {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };
    pub const Z: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };

    #[inline(always)]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Vec3 { x, y, z }
    }
    #[inline(always)]
    pub fn dot(self, o: Vec3) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    #[inline(always)]
    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
    #[inline(always)]
    pub fn length_squared(self) -> f32 {
        self.dot(self)
    }
    #[inline(always)]
    pub fn length(self) -> f32 {
        fm::sqrt(self.length_squared())
    }
    /// Unit vector in the same direction, or zero if the length is (almost) zero.
    #[inline(always)]
    pub fn normalized(self) -> Vec3 {
        let l2 = self.length_squared();
        if l2 > 1e-30 {
            self * (1.0 / fm::sqrt(l2))
        } else {
            Vec3::ZERO
        }
    }
    #[inline(always)]
    pub fn distance(self, o: Vec3) -> f32 {
        (self - o).length()
    }
    #[inline(always)]
    pub fn lerp(self, o: Vec3, t: f32) -> Vec3 {
        self + (o - self) * t
    }
    /// Any unit vector perpendicular to `self` (which must be non-zero).
    pub fn any_perpendicular(self) -> Vec3 {
        let a = if self.x.abs() < 0.9 * self.length().max(1e-20) {
            Vec3::X
        } else {
            Vec3::Y
        };
        self.cross(a).normalized()
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    #[inline(always)]
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl Sub for Vec3 {
    type Output = Vec3;
    #[inline(always)]
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Mul<f32> for Vec3 {
    type Output = Vec3;
    #[inline(always)]
    fn mul(self, s: f32) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}
impl Mul<Vec3> for f32 {
    type Output = Vec3;
    #[inline(always)]
    fn mul(self, v: Vec3) -> Vec3 {
        v * self
    }
}
impl Div<f32> for Vec3 {
    type Output = Vec3;
    #[inline(always)]
    fn div(self, s: f32) -> Vec3 {
        self * (1.0 / s)
    }
}
impl Neg for Vec3 {
    type Output = Vec3;
    #[inline(always)]
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}
impl AddAssign for Vec3 {
    #[inline(always)]
    fn add_assign(&mut self, o: Vec3) {
        *self = *self + o;
    }
}
impl SubAssign for Vec3 {
    #[inline(always)]
    fn sub_assign(&mut self, o: Vec3) {
        *self = *self - o;
    }
}
impl MulAssign<f32> for Vec3 {
    #[inline(always)]
    fn mul_assign(&mut self, s: f32) {
        *self = *self * s;
    }
}

/// A rotation quaternion (`x, y, z` imaginary part, `w` real part).
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Default for Quat {
    fn default() -> Self {
        Quat::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    #[inline(always)]
    pub const fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Quat { x, y, z, w }
    }

    /// Rotation of `angle` radians around the (unit) `axis`.
    pub fn from_axis_angle(axis: Vec3, angle: f32) -> Quat {
        let a = axis.normalized();
        let (s, c) = (fm::sin(angle * 0.5), fm::cos(angle * 0.5));
        Quat::new(a.x * s, a.y * s, a.z * s, c)
    }

    /// Shortest-arc rotation taking direction `from` onto direction `to`.
    /// Handles parallel and anti-parallel vectors. Inputs need not be normalized.
    pub fn from_to(from: Vec3, to: Vec3) -> Quat {
        let (f, t) = (from.normalized(), to.normalized());
        if f == Vec3::ZERO || t == Vec3::ZERO {
            return Quat::IDENTITY;
        }
        let d = f.dot(t);
        if d > 0.999_999 {
            return Quat::IDENTITY;
        }
        if d < -0.999_999 {
            let p = f.any_perpendicular();
            return Quat::new(p.x, p.y, p.z, 0.0);
        }
        let c = f.cross(t);
        Quat::new(c.x, c.y, c.z, 1.0 + d).normalized()
    }

    #[inline(always)]
    pub fn dot(self, o: Quat) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z + self.w * o.w
    }
    #[inline(always)]
    pub fn conjugate(self) -> Quat {
        Quat::new(-self.x, -self.y, -self.z, self.w)
    }
    #[inline(always)]
    pub fn length(self) -> f32 {
        fm::sqrt(self.dot(self))
    }
    pub fn normalized(self) -> Quat {
        let l = self.length();
        if l > 1e-15 {
            let i = 1.0 / l;
            Quat::new(self.x * i, self.y * i, self.z * i, self.w * i)
        } else {
            Quat::IDENTITY
        }
    }

    /// Rotate a vector.
    #[inline(always)]
    pub fn rotate(self, v: Vec3) -> Vec3 {
        // v + 2w(q×v) + 2 q×(q×v)
        let q = Vec3::new(self.x, self.y, self.z);
        let t = q.cross(v) * 2.0;
        v + t * self.w + q.cross(t)
    }

    /// Flips the sign so that `w >= 0` (q and -q are the same rotation).
    pub fn with_positive_w(self) -> Quat {
        if self.w < 0.0 {
            Quat::new(-self.x, -self.y, -self.z, -self.w)
        } else {
            self
        }
    }

    /// Normalized linear interpolation (shortest path).
    pub fn nlerp(self, o: Quat, t: f32) -> Quat {
        let o = if self.dot(o) < 0.0 {
            Quat::new(-o.x, -o.y, -o.z, -o.w)
        } else {
            o
        };
        Quat::new(
            self.x + (o.x - self.x) * t,
            self.y + (o.y - self.y) * t,
            self.z + (o.z - self.z) * t,
            self.w + (o.w - self.w) * t,
        )
        .normalized()
    }

    /// Angle of the rotation in radians, in `[0, π]`.
    pub fn angle(self) -> f32 {
        let q = self.normalized().with_positive_w();
        2.0 * fm::acos(q.w.clamp(-1.0, 1.0))
    }
}

impl Mul for Quat {
    type Output = Quat;
    /// Hamilton product: `(a * b).rotate(v) == a.rotate(b.rotate(v))`.
    #[inline(always)]
    fn mul(self, o: Quat) -> Quat {
        Quat::new(
            self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
            self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Vec3, b: Vec3) -> bool {
        (a - b).length() < 1e-4
    }

    #[test]
    fn vec3_basics() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(-2.0, 0.5, 4.0);
        assert_eq!(a + b, Vec3::new(-1.0, 2.5, 7.0));
        assert_eq!(a.dot(b), -2.0 + 1.0 + 12.0);
        assert!(close(
            a.cross(b),
            Vec3::new(
                2.0 * 4.0 - 3.0 * 0.5,
                3.0 * -2.0 - 1.0 * 4.0,
                1.0 * 0.5 - 2.0 * -2.0
            )
        ));
        assert!((Vec3::new(3.0, 4.0, 0.0).length() - 5.0).abs() < 1e-6);
        assert_eq!(Vec3::ZERO.normalized(), Vec3::ZERO);
        assert!((a.normalized().length() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn perpendicular_is_perpendicular() {
        for v in [
            Vec3::X,
            Vec3::Y,
            Vec3::Z,
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(-0.3, 5.0, 0.1),
        ] {
            let p = v.any_perpendicular();
            assert!(v.dot(p).abs() < 1e-5 * v.length());
            assert!((p.length() - 1.0).abs() < 1e-5);
        }
    }

    #[test]
    fn quat_rotates_like_axis_angle() {
        let q = Quat::from_axis_angle(Vec3::Z, core::f32::consts::FRAC_PI_2);
        assert!(close(q.rotate(Vec3::X), Vec3::Y));
        assert!(close(q.rotate(Vec3::Y), -Vec3::X));
    }

    #[test]
    fn quat_product_composes_rotations() {
        let a = Quat::from_axis_angle(Vec3::Z, 0.7);
        let b = Quat::from_axis_angle(Vec3::X, -1.1);
        let v = Vec3::new(0.3, -1.2, 2.0);
        assert!(close((a * b).rotate(v), a.rotate(b.rotate(v))));
        assert!(close((a * a.conjugate()).rotate(v), v));
    }

    #[test]
    fn from_to_maps_direction() {
        let pairs = [
            (Vec3::X, Vec3::Y),
            (Vec3::new(1.0, 2.0, 3.0), Vec3::new(-3.0, 0.5, 1.0)),
            (Vec3::X, -Vec3::X),      // anti-parallel
            (Vec3::Y, Vec3::Y * 3.0), // parallel
        ];
        for (f, t) in pairs {
            let q = Quat::from_to(f, t);
            assert!(
                close(q.rotate(f.normalized()), t.normalized()),
                "{f:?} -> {t:?}"
            );
        }
    }

    #[test]
    fn quat_angle() {
        assert!((Quat::from_axis_angle(Vec3::Y, 1.2).angle() - 1.2).abs() < 1e-4);
        assert!(Quat::IDENTITY.angle() < 1e-6);
    }
}
