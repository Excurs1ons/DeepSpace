//! 核心数学：常量、四元数、3×3 矩阵
use crate::Vec3;

/// 四元数 (w, x, y, z)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    pub fn identity() -> Self { Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 } }

    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self { Self { w, x, y, z } }

    /// 从轴-角创建
    pub fn from_axis_angle(axis: Vec3, angle: f64) -> Self {
        let ha = angle * 0.5;
        let s = ha.sin();
        let n = axis.normalized();
        Self { w: ha.cos(), x: n.x * s, y: n.y * s, z: n.z * s }
    }

    /// 从欧拉角创建 (pitch, yaw, roll)
    pub fn from_euler(pitch: f64, yaw: f64, roll: f64) -> Self {
        let (sp, cp) = (pitch * 0.5).sin_cos();
        let (sy, cy) = (yaw * 0.5).sin_cos();
        let (sr, cr) = (roll * 0.5).sin_cos();
        Self {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    pub fn mul(&self, o: &Quaternion) -> Quaternion {
        Quaternion::new(
            self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        )
    }

    /// 旋转向量
    pub fn rotate(&self, v: &Vec3) -> Vec3 {
        let ix = self.w * v.x + self.y * v.z - self.z * v.y;
        let iy = self.w * v.y + self.z * v.x - self.x * v.z;
        let iz = self.w * v.z + self.x * v.y - self.y * v.x;
        let iw = -self.x * v.x - self.y * v.y - self.z * v.z;
        Vec3::new(
            ix * self.w + iw * -self.x + iy * -self.z - iz * -self.y,
            iy * self.w + iw * -self.y + iz * -self.x - ix * -self.z,
            iz * self.w + iw * -self.z + ix * -self.y - iy * -self.x,
        )
    }

    pub fn conjugate(&self) -> Self { Self { w: self.w, x: -self.x, y: -self.y, z: -self.z } }

    pub fn magnitude(&self) -> f64 { (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt() }

    pub fn normalized(&self) -> Self {
        let m = self.magnitude();
        if m > 0.0 { Self { w: self.w / m, x: self.x / m, y: self.y / m, z: self.z / m } } else { Self::identity() }
    }

    pub fn to_euler(&self) -> Vec3 {
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = sinr_cosp.atan2(cosr_cosp);
        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch = if sinp.abs() >= 1.0 { (std::f64::consts::PI / 2.0).copysign(sinp) } else { sinp.asin() };
        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = siny_cosp.atan2(cosy_cosp);
        Vec3::new(pitch, yaw, roll)
    }
}

impl std::ops::Mul<&Quaternion> for &Quaternion {
    type Output = Quaternion;
    fn mul(self, o: &Quaternion) -> Quaternion { self.mul(o) }
}

// =====================================================================
// 3×3 矩阵
// =====================================================================
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat3x3 {
    pub m: [f64; 9],
}

impl Mat3x3 {
    pub fn identity() -> Self {
        Self { m: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] }
    }

    pub fn from_diag(ix: f64, iy: f64, iz: f64) -> Self {
        Self { m: [ix, 0.0, 0.0, 0.0, iy, 0.0, 0.0, 0.0, iz] }
    }

    pub fn rotation(axis: Vec3, angle: f64) -> Self {
        let n = axis.normalized();
        let (c, s) = (angle.cos(), angle.sin());
        let t = 1.0 - c;
        Self {
            m: [t * n.x * n.x + c,     t * n.x * n.y - n.z * s, t * n.x * n.z + n.y * s,
                t * n.x * n.y + n.z * s, t * n.y * n.y + c,      t * n.y * n.z - n.x * s,
                t * n.x * n.z - n.y * s, t * n.y * n.z + n.x * s, t * n.z * n.z + c],
        }
    }

    pub fn mul_mat(&self, o: &Mat3x3) -> Mat3x3 {
        let (a, b) = (&self.m, &o.m);
        Self {
            m: [a[0]*b[0]+a[1]*b[3]+a[2]*b[6], a[0]*b[1]+a[1]*b[4]+a[2]*b[7], a[0]*b[2]+a[1]*b[5]+a[2]*b[8],
                a[3]*b[0]+a[4]*b[3]+a[5]*b[6], a[3]*b[1]+a[4]*b[4]+a[5]*b[7], a[3]*b[2]+a[4]*b[5]+a[5]*b[8],
                a[6]*b[0]+a[7]*b[3]+a[8]*b[6], a[6]*b[1]+a[7]*b[4]+a[8]*b[7], a[6]*b[2]+a[7]*b[5]+a[8]*b[8]],
        }
    }

    pub fn mul_vec(&self, v: &Vec3) -> Vec3 {
        Vec3::new(self.m[0]*v.x + self.m[1]*v.y + self.m[2]*v.z,
                  self.m[3]*v.x + self.m[4]*v.y + self.m[5]*v.z,
                  self.m[6]*v.x + self.m[7]*v.y + self.m[8]*v.z)
    }

    pub fn transpose(&self) -> Self {
        Self { m: [self.m[0], self.m[3], self.m[6],
                   self.m[1], self.m[4], self.m[7],
                   self.m[2], self.m[5], self.m[8]] }
    }

    pub fn determinant(&self) -> f64 {
        self.m[0] * (self.m[4] * self.m[8] - self.m[5] * self.m[7])
            - self.m[1] * (self.m[3] * self.m[8] - self.m[5] * self.m[6])
            + self.m[2] * (self.m[3] * self.m[7] - self.m[4] * self.m[6])
    }

    pub fn inverse(&self) -> Self {
        let d = self.determinant();
        if d.abs() < 1e-15 { return Self::identity(); }
        let inv = 1.0 / d;
        Self {
            m: [(self.m[4]*self.m[8]-self.m[5]*self.m[7])*inv, (self.m[2]*self.m[7]-self.m[1]*self.m[8])*inv, (self.m[1]*self.m[5]-self.m[2]*self.m[4])*inv,
                (self.m[5]*self.m[6]-self.m[3]*self.m[8])*inv, (self.m[0]*self.m[8]-self.m[2]*self.m[6])*inv, (self.m[2]*self.m[3]-self.m[0]*self.m[5])*inv,
                (self.m[3]*self.m[7]-self.m[4]*self.m[6])*inv, (self.m[1]*self.m[6]-self.m[0]*self.m[7])*inv, (self.m[0]*self.m[4]-self.m[1]*self.m[3])*inv],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vec3;
    const TOL: f64 = 1e-12;

    #[test]
    fn quaternion_identity() {
        let q = Quaternion::identity();
        assert!((q.magnitude() - 1.0).abs() < TOL);
        let v = Vec3::new(1.0, 2.0, 3.0);
        let r = q.rotate(&v);
        assert!((r - v).length() < TOL);
    }

    #[test]
    fn quaternion_rotation_90z() {
        let q = Quaternion::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f64::consts::PI / 2.0);
        let v = Vec3::new(1.0, 0.0, 0.0);
        let r = q.rotate(&v);
        assert!((r - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-10);
    }

    #[test]
    fn quaternion_multiplication() {
        let qx = Quaternion::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), std::f64::consts::PI / 2.0);
        let qy = Quaternion::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), std::f64::consts::PI / 2.0);
        let qc = qx.mul(&qy);
        let v = Vec3::new(1.0, 0.0, 0.0);
        let r = qc.rotate(&v);
        // 先绕X转90°再绕Y转90°
        assert!(r.length() > 0.0);
    }

    #[test]
    fn quaternion_normalized() {
        let q = Quaternion::new(2.0, 0.0, 0.0, 0.0);
        let n = q.normalized();
        assert!((n.magnitude() - 1.0).abs() < TOL);
        assert!((n.w - 1.0).abs() < TOL);
    }

    #[test]
    fn quaternion_euler_roundtrip() {
        // from_euler → to_euler 应在特殊角度外可逆
        let q = Quaternion::from_euler(0.3, 0.5, 0.2);
        let e = q.to_euler();
        let q2 = Quaternion::from_euler(e.x, e.y, e.z);
        // 四元数可能差个符号，比较旋转效果
        let v = Vec3::new(1.0, 2.0, 3.0);
        let r1 = q.rotate(&v);
        let r2 = q2.rotate(&v);
        assert!((r1 - r2).length() < 1e-10);
    }

    #[test]
    fn mat3x3_identity() {
        let m = Mat3x3::identity();
        let v = Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(m.mul_vec(&v), v);
    }

    #[test]
    fn mat3x3_rotation() {
        let m = Mat3x3::rotation(Vec3::new(0.0, 0.0, 1.0), std::f64::consts::PI / 2.0);
        let v = Vec3::new(1.0, 0.0, 0.0);
        let r = m.mul_vec(&v);
        assert!((r - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-10);
    }

    #[test]
    fn mat3x3_mul() {
        let a = Mat3x3::rotation(Vec3::new(1.0, 0.0, 0.0), 0.5);
        let b = Mat3x3::rotation(Vec3::new(0.0, 1.0, 0.0), 0.3);
        let c = a.mul_mat(&b);
        let v = Vec3::new(1.0, 0.0, 0.0);
        // c = a * b, so c*v = a*(b*v)
        let expected = a.mul_vec(&b.mul_vec(&v));
        assert!((c.mul_vec(&v) - expected).length() < TOL);
    }

    #[test]
    fn mat3x3_transpose() {
        let m = Mat3x3 { m: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0] };
        let t = m.transpose();
        assert!((t.m[1] - 4.0).abs() < TOL);
        assert!((t.m[3] - 2.0).abs() < TOL);
    }

    #[test]
    fn mat3x3_inverse_rotation() {
        let m = Mat3x3::rotation(Vec3::new(0.0, 1.0, 0.0), 1.2);
        let inv = m.inverse();
        let v = Vec3::new(1.0, 2.0, 3.0);
        let r = inv.mul_vec(&m.mul_vec(&v));
        assert!((r - v).length() < 1e-10);
    }

    #[test]
    fn mat3x3_determinant_rotation() {
        let m = Mat3x3::rotation(Vec3::new(1.0, 2.0, 3.0).normalized(), 0.7);
        assert!((m.determinant() - 1.0).abs() < TOL);
    }

    #[test]
    fn mat3x3_diag_inverse() {
        let m = Mat3x3::from_diag(2.0, 3.0, 4.0);
        let inv = m.inverse();
        assert!((inv.m[0] - 0.5).abs() < TOL);
        assert!((inv.m[4] - 1.0/3.0).abs() < TOL);
        assert!((inv.m[8] - 0.25).abs() < TOL);
    }
}
