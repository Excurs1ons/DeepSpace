//! 传感器（恢复占位实现）
//!
//! ⚠ 此文件为编译占位：原完整实现未纳入 git 且工作区缺失。
//! 当前仅提供 world.rs 引用的 `Radar`/`RadarConfig`/`RadarMode` 最小接口。

/// 雷达工作模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RadarMode {
    Search,
    Stt,
}

/// 雷达配置
#[derive(Debug, Clone)]
pub struct RadarConfig {
    /// 最大探测距离 (m)
    pub range_m: f64,
    /// 峰值功率 (kW)
    pub peak_power_kw: f64,
    /// 天线孔径 (m²)
    pub aperture_m2: f64,
    /// 噪声系数
    pub noise_figure_db: f64,
}

impl RadarConfig {
    /// APG-77 类机载雷达典型参数
    pub fn apg77() -> Self {
        RadarConfig {
            range_m: 400_000.0,
            peak_power_kw: 16.0,
            aperture_m2: 0.6,
            noise_figure_db: 4.0,
        }
    }
}

/// 拦截雷达：对弹道目标做简易探测概率估计
#[derive(Debug, Clone)]
pub struct Radar {
    pub config: RadarConfig,
    pub mode: RadarMode,
}

impl Radar {
    pub fn new(config: RadarConfig) -> Self {
        Radar {
            config,
            mode: RadarMode::Search,
        }
    }

    pub fn set_mode(&mut self, mode: RadarMode) {
        self.mode = mode;
    }

    /// 探测概率：SNR ∝ rcs / r⁴ 的简化雷达方程 + logistic 平滑
    /// `range_m` — 目标距离 (m)；`rcs_m2` — 目标雷达截面积 (m²)
    pub fn detection_probability(&self, range_m: f64, rcs_m2: f64) -> f64 {
        let r = range_m.max(1.0);
        let r0 = self.config.range_m;
        // 参考距离处 1 m² 目标 SNR = 20 dB 起，按 r⁻⁴ 衰减
        let snr_db = 20.0 + 10.0 * (rcs_m2.max(1e-4) / 1.0).log10()
            - 40.0 * (r / r0).max(1e-6).log10();
        let pd = 1.0 / (1.0 + (-(snr_db - 10.0) / 3.0).exp());
        pd.clamp(0.0, 1.0)
    }
}
