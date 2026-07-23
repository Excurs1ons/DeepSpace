pub mod frame_graph;
pub mod core;
pub mod environment;
pub mod physics;
pub mod mission;
pub mod vessel;
pub mod app;

// =====================================================================
// 基础 3D 向量
// =====================================================================
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self { Self { x, y, z } }
    pub fn zero() -> Self { Self { x: 0.0, y: 0.0, z: 0.0 } }
    pub fn length(&self) -> f64 { (self.x * self.x + self.y * self.y + self.z * self.z).sqrt() }
    pub fn length_squared(&self) -> f64 { self.x * self.x + self.y * self.y + self.z * self.z }
    pub fn normalized(&self) -> Self {
        let l = self.length();
        if l > 0.0 { Self { x: self.x / l, y: self.y / l, z: self.z / l } } else { Self::zero() }
    }
    pub fn dot(&self, o: &Self) -> f64 { self.x * o.x + self.y * o.y + self.z * o.z }
    pub fn cross(&self, o: &Self) -> Self {
        Self {
            x: self.y * o.z - self.z * o.y,
            y: self.z * o.x - self.x * o.z,
            z: self.x * o.y - self.y * o.x,
        }
    }
}

impl std::ops::Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self { Self { x: -self.x, y: -self.y, z: -self.z } }
}

impl std::ops::Neg for &Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 { Vec3 { x: -self.x, y: -self.y, z: -self.z } }
}

impl std::ops::Add for Vec3 {
    type Output = Self;
    fn add(self, o: Self) -> Self { Self { x: self.x + o.x, y: self.y + o.y, z: self.z + o.z } }
}

impl std::ops::Add<&Vec3> for Vec3 {
    type Output = Self;
    fn add(self, o: &Vec3) -> Self { Self { x: self.x + o.x, y: self.y + o.y, z: self.z + o.z } }
}

impl std::ops::Add<Vec3> for &Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 { Vec3 { x: self.x + o.x, y: self.y + o.y, z: self.z + o.z } }
}

impl std::ops::AddAssign for Vec3 {
    fn add_assign(&mut self, o: Self) { self.x += o.x; self.y += o.y; self.z += o.z; }
}

impl std::ops::Sub for Vec3 {
    type Output = Self;
    fn sub(self, o: Self) -> Self { Self { x: self.x - o.x, y: self.y - o.y, z: self.z - o.z } }
}

impl std::ops::Sub<&Vec3> for Vec3 {
    type Output = Self;
    fn sub(self, o: &Vec3) -> Self { Self { x: self.x - o.x, y: self.y - o.y, z: self.z - o.z } }
}

impl std::ops::Sub<Vec3> for &Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 { Vec3 { x: self.x - o.x, y: self.y - o.y, z: self.z - o.z } }
}

impl std::ops::Mul<f64> for Vec3 {
    type Output = Self;
    fn mul(self, s: f64) -> Self { Self { x: self.x * s, y: self.y * s, z: self.z * s } }
}

impl std::ops::Mul<Vec3> for f64 {
    type Output = Vec3;
    fn mul(self, v: Vec3) -> Vec3 { Vec3 { x: self * v.x, y: self * v.y, z: self * v.z } }
}

impl std::ops::Div<f64> for Vec3 {
    type Output = Self;
    fn div(self, s: f64) -> Self { Self { x: self.x / s, y: self.y / s, z: self.z / s } }
}

// =====================================================================
// 物理常量
// =====================================================================
pub const G: f64 = 6.674_30e-11;
pub const G0: f64 = 9.80665;

// 空气常数 (environment.rs 使用)
pub const AIR_GAS_CONSTANT: f64 = 287.058;   // J/(kg·K), 干空气气体常数
pub const GAMMA_AIR: f64 = 1.4;              // 绝热指数比热容比
