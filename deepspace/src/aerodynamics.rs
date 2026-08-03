//! 大气/风场（恢复占位实现）
//!
//! ⚠ 此文件为编译占位：原完整实现未纳入 git 且工作区缺失。
//! 当前仅提供 missile.rs 引用的 `WindField` 最小接口。

use crate::Vec3;

/// 简化高空风场：风速随高度分段（地表风 → 急流 → 高空静风）
#[derive(Debug, Clone, Copy, Default)]
pub struct WindField {
    /// 地面风速 (m/s)，沿 +X 方向
    pub surface_speed: f64,
}

impl WindField {
    /// 返回高度 z (m) 处的风矢量 (m/s)
    pub fn wind_at(&self, z: f64) -> Vec3 {
        let h = z.max(0.0);
        let speed = if h > 20_000.0 {
            0.0 // 平流层以上静风
        } else if h > 8_000.0 {
            // 急流带：线性衰减到 0
            self.surface_speed * (1.0 - (h - 8_000.0) / 12_000.0)
        } else {
            // 对流层：随高度缓慢增强
            self.surface_speed * (1.0 + 0.05 * h / 8_000.0)
        };
        Vec3::new(speed, 0.0, 0.0)
    }
}
