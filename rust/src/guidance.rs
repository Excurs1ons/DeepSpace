//! 模块化飞控计算机系统
//!
//! 制导算法可插拔设计：
//! - `GuidanceAlgorithm` trait 定义制导接口
//! - `FlightComputer` 是"飞控盒"，持有制导算法 + 状态
//! - 各算法在单独 impl 中实现，通过配置文件切换
//!
//! 当前算法：
//! - `CosineGuidance` — 余弦重力转弯 (原 app.rs 硬编码逻辑)
//! - `PEGGuidance` — 迭代制导 (待实现)

use crate::Vec3;

// =====================================================================
// 飞控输入状态
// =====================================================================
/// 飞控每帧的输入——制导算法计算推力方向所需的全部信息
#[derive(Debug, Clone)]
pub struct GuidanceState {
    /// 海拔高度 (m)
    pub altitude: f64,
    /// 速度大小 (m/s)
    pub velocity_mag: f64,
    /// 位置矢量 (ECEF)
    pub position: Vec3,
    /// 速度矢量 (ECEF)
    pub velocity: Vec3,
    /// 任务时间 (s)
    pub mission_time: f64,
    /// 当前总质量 (kg)
    pub total_mass_kg: f64,
    /// 当前级数 (0 = 第一级)
    pub stage: i32,
    /// 当前油门 (0.0~1.0)
    pub throttle: f64,
}

// =====================================================================
// 制导输出指令
// =====================================================================
/// 飞控输出——推力方向 + 油门
#[derive(Debug, Clone)]
pub struct SteeringCommand {
    /// 推力方向单位矢量 (ECEF)
    pub thrust_direction: Vec3,
    /// 油门 (0.0 = 关机, 1.0 = 满推)
    pub throttle: f64,
}

impl Default for SteeringCommand {
    fn default() -> Self {
        SteeringCommand {
            thrust_direction: Vec3::new(0.0, 1.0, 0.0), // 指向上方
            throttle: 1.0,
        }
    }
}

// =====================================================================
// 制导算法 trait
// =====================================================================
/// 任何制导算法必须实现此 trait
///
/// # 约定
/// - `reset()` 在新任务开始时调用
/// - `compute()` 每仿真步被调用一次
/// - 返回值中的 `throttle` 覆盖发动机当前油门
pub trait GuidanceAlgorithm: std::fmt::Debug + Send {
    /// 根据当前状态和配置计算推力指令
    fn compute(&mut self, state: &GuidanceState, config: &GuidanceConfig) -> SteeringCommand;

    /// 重置算法内部状态（新任务）
    fn reset(&mut self);
}

// =====================================================================
// 制导配置
// =====================================================================
/// 所有制导算法共用的配置参数 + 算法选择
#[derive(Debug, Clone)]
pub struct GuidanceConfig {
    /// 制导算法名（"cosine", "peg" 等）
    pub algorithm: String,

    // --- 余弦转弯参数 ---
    /// 开始俯仰高度 (m)
    pub pitch_start_alt_m: f64,
    /// 结束俯仰高度 (m)
    pub pitch_end_alt_m: f64,
    /// 终点俯仰角（度，0=垂直，90=水平）
    pub pitch_end_angle_deg: f64,

    // --- 通用参数 ---
    /// 轨道容差 (m)
    pub orbit_tolerance_m: f64,
}

impl Default for GuidanceConfig {
    fn default() -> Self {
        GuidanceConfig {
            algorithm: "cosine".into(),
            pitch_start_alt_m: 2_000.0,
            pitch_end_alt_m: 20_000.0,
            pitch_end_angle_deg: 85.0,
            orbit_tolerance_m: 10_000.0,
        }
    }
}

// =====================================================================
// 飞控计算机
// =====================================================================
/// FlightComputer — "飞控盒"
///
/// 职责：
/// 1. 持有当前制导算法
/// 2. 每帧将传感器状态传给算法，输出指令
/// 3. 处理算法切换/重置
///
/// # 示例
/// ```ignore
/// let mut fc = FlightComputer::new();
/// fc.set_algorithm(Box::new(CosineGuidance::new()));
/// let cmd = fc.update(&state);
/// vessel.body.set_orientation_from_dir(cmd.thrust_direction);
/// ```
#[derive(Debug)]
pub struct FlightComputer {
    pub config: GuidanceConfig,
    algorithm: Box<dyn GuidanceAlgorithm>,
}

impl FlightComputer {
    /// 使用默认配置和余弦转弯算法创建飞控
    pub fn new() -> Self {
        FlightComputer {
            config: GuidanceConfig::default(),
            algorithm: Box::new(CosineGuidance::new()),
        }
    }

    /// 从配置创建飞控——根据 `config.algorithm` 选择算法
    pub fn from_config(config: &GuidanceConfig) -> Self {
        let algorithm = Self::create_algorithm(&config.algorithm);
        FlightComputer {
            config: config.clone(),
            algorithm,
        }
    }

    /// 根据算法名创建对应实例
    fn create_algorithm(name: &str) -> Box<dyn GuidanceAlgorithm> {
        match name {
            "cosine" => Box::new(CosineGuidance::new()),
            "peg" => Box::new(PEGGuidance::new()),
            _ => {
                eprintln!("  WARNING: Unknown guidance algorithm '{}', falling back to cosine", name);
                Box::new(CosineGuidance::new())
            }
        }
    }

    /// 每帧调用——传入传感器状态，返回制导指令
    pub fn update(&mut self, state: &GuidanceState) -> SteeringCommand {
        self.algorithm.compute(state, &self.config)
    }

    /// 运行时更换算法（热切换）
    pub fn set_algorithm(&mut self, name: &str) {
        self.config.algorithm = name.into();
        self.algorithm = Self::create_algorithm(name);
    }

    /// 重置飞控（新任务）
    pub fn reset(&mut self) {
        self.algorithm.reset();
    }
}

// =====================================================================
// CosineGuidance — 余弦重力转弯
// =====================================================================
/// 余弦重力转弯
///
/// 从 `pitch_start_alt_m` 到 `pitch_end_alt_m` 按余弦曲线从垂直转向水平。
/// 起点导数为零，平滑过渡。终点接近水平。
///
/// ```
/// pitch(°) = 90 × (1 - cos(progress × π/2))
/// ```
///
/// 这是从 app.rs 硬编码逻辑提取的独立实现，保持完全相同的行为。
#[derive(Debug, Clone)]
pub struct CosineGuidance;

impl CosineGuidance {
    pub fn new() -> Self {
        CosineGuidance
    }
}

impl GuidanceAlgorithm for CosineGuidance {
    fn compute(&mut self, state: &GuidanceState, config: &GuidanceConfig) -> SteeringCommand {
        let p_start = config.pitch_start_alt_m;
        let p_end = config.pitch_end_alt_m;

        if p_end > p_start && state.altitude >= p_start && state.altitude < p_end {
            // 余弦曲线俯仰段
            let progress = (state.altitude - p_start) / (p_end - p_start);
            let angle_rad = (std::f64::consts::PI / 2.0 * progress).cos();
            let pitch_end_rad = config.pitch_end_angle_deg * std::f64::consts::PI / 180.0;
            let pitch_rad = pitch_end_rad * (1.0 - angle_rad);
            // sin(pitch)=水平分量, cos(pitch)=垂直分量
            let dir = Vec3::new(pitch_rad.sin(), pitch_rad.cos(), 0.0);
            SteeringCommand {
                thrust_direction: dir,
                throttle: 1.0,
            }
        } else if state.altitude >= p_end && p_end > p_start {
            // 俯仰完成 → 保持终点俯仰角
            let pitch_end_rad = config.pitch_end_angle_deg * std::f64::consts::PI / 180.0;
            let dir = Vec3::new(pitch_end_rad.sin(), pitch_end_rad.cos(), 0.0);
            SteeringCommand {
                thrust_direction: dir,
                throttle: 1.0,
            }
        } else {
            // 低于转弯起始高度 → 垂直上升
            SteeringCommand::default()
        }
    }

    fn reset(&mut self) {
        // CosineGuidance 无内部状态
    }
}

// =====================================================================
// PEGGuidance — 迭代制导
// =====================================================================
/// 迭代制导（PEG — Powered Explicit Guidance 简化版）
///
/// 双阶段制导策略：
/// 1. **核心级（stage 0）**：同余弦转弯，从 pitch_start 到 pitch_end 平滑过渡
/// 2. **上面级（stage ≥ 1）**：沿速度方向推进（prograde），用于远地点圆化燃烧
///
/// 配合 mission.rs 的滑行-远地点点火序列使用：
/// - MECO → 滑行至远地点 → ICPS prograde 点火 → 圆化断油
///
/// # 为何 prograde
/// 远地点圆化燃烧的最优方向就是速度方向——完全沿瞬时速度矢量加速，
/// 以最小 Δv 将 periapsis 提升到目标值，同时 apoapsis 略微上升。
#[derive(Debug, Clone)]
pub struct PEGGuidance;

impl PEGGuidance {
    pub fn new() -> Self {
        PEGGuidance
    }
}

impl GuidanceAlgorithm for PEGGuidance {
    fn compute(&mut self, state: &GuidanceState, config: &GuidanceConfig) -> SteeringCommand {
        // ── 阶段 1：核心级（stage 0）—— 余弦重力转弯 ──
        if state.stage == 0 {
            let p_start = config.pitch_start_alt_m;
            let p_end = config.pitch_end_alt_m;

            if p_end > p_start && state.altitude >= p_start && state.altitude < p_end {
                // 余弦曲线俯仰段
                let progress = (state.altitude - p_start) / (p_end - p_start);
                let angle_rad = (std::f64::consts::PI / 2.0 * progress).cos();
                let pitch_end_rad = config.pitch_end_angle_deg * std::f64::consts::PI / 180.0;
                let pitch_rad = pitch_end_rad * (1.0 - angle_rad);
                // sin(pitch)=水平分量, cos(pitch)=垂直分量
                let dir = Vec3::new(pitch_rad.sin(), pitch_rad.cos(), 0.0);
                return SteeringCommand {
                    thrust_direction: dir,
                    throttle: 1.0,
                };
            } else if state.altitude >= p_end && p_end > p_start {
                // 俯仰完成 → 保持终点俯仰角
                let pitch_end_rad = config.pitch_end_angle_deg * std::f64::consts::PI / 180.0;
                let dir = Vec3::new(pitch_end_rad.sin(), pitch_end_rad.cos(), 0.0);
                return SteeringCommand {
                    thrust_direction: dir,
                    throttle: 1.0,
                };
            } else {
                // 低于转弯起始高度 → 垂直上升
                return SteeringCommand::default();
            }
        }

        // ── 阶段 2：上面级（stage ≥ 1）—— Prograde 制导 ──
        // 沿速度方向推进，用于远地点圆化燃烧
        let vel = state.velocity;
        let vel_mag = vel.length();
        if vel_mag > 1.0 {
            let dir = vel / vel_mag;
            SteeringCommand {
                thrust_direction: dir,
                throttle: state.throttle, // 由 mission.rs 控制油门
            }
        } else {
            // 速度太小时保底——沿水平方向
            SteeringCommand {
                thrust_direction: Vec3::new(1.0, 0.0, 0.0),
                throttle: 0.0,
            }
        }
    }

    fn reset(&mut self) {
        // PEGGuidance 无内部状态
    }
}

// =====================================================================
// 测试
// =====================================================================
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cosine_guidance_vertical_before_start() {
        let mut guid = CosineGuidance::new();
        let config = GuidanceConfig::default(); // pitch_start=2000
        let state = GuidanceState {
            altitude: 1000.0,
            ..dummy_state()
        };
        let cmd = guid.compute(&state, &config);
        // 低于 2000m → 垂直向上
        assert!(cmd.thrust_direction.y > 0.99, "should be nearly vertical");
        assert_eq!(cmd.throttle, 1.0);
    }

    #[test]
    fn test_cosine_guidance_horizontal_at_end() {
        let mut guid = CosineGuidance::new();
        let config = GuidanceConfig::default(); // pitch_end=20000, pitch_end_angle=85°
        // 超过 pitch_end → 保持终点俯仰角 (85°)
        let state = GuidanceState {
            altitude: 30000.0,
            ..dummy_state()
        };
        let cmd = guid.compute(&state, &config);
        let pitch_end_rad = 85.0_f64 * std::f64::consts::PI / 180.0;
        let expected = Vec3::new(pitch_end_rad.sin(), pitch_end_rad.cos(), 0.0);
        assert!(
            (cmd.thrust_direction - expected).length() < 0.01,
            "should hold final pitch angle past pitch_end"
        );
        let state = GuidanceState {
            altitude: 10_000.0, // midway: 2000→20000
            ..dummy_state()
        };
        let cmd = guid.compute(&state, &config);
        // 应该有水平和垂直分量
        assert!(cmd.thrust_direction.length() > 0.99, "should be unit vector");
        assert!(cmd.thrust_direction.x.abs() > 0.1, "should have some horizontal component");
        assert!(cmd.thrust_direction.y > 0.3, "should have some vertical component");
    }

    #[test]
    fn test_flight_computer_new() {
        let fc = FlightComputer::new();
        assert_eq!(fc.config.algorithm, "cosine");
    }

    #[test]
    fn test_flight_computer_from_config() {
        let cfg = GuidanceConfig {
            algorithm: "cosine".into(),
            ..Default::default()
        };
        let mut fc = FlightComputer::from_config(&cfg);
        let cmd = fc.update(&dummy_state());
        // 默认行为：垂直上升（低于 2000m）
        assert!(cmd.thrust_direction.y > 0.99);
    }

    #[test]
    fn test_flight_computer_unknown_fallback() {
        let cfg = GuidanceConfig {
            algorithm: "nonexistent".into(),
            ..Default::default()
        };
        let mut fc = FlightComputer::from_config(&cfg);
        let cmd = fc.update(&dummy_state());
        // 应 fallback 到 cosine
        assert!(cmd.thrust_direction.y > 0.99);
    }

    #[test]
    fn test_flight_computer_reset() {
        let mut fc = FlightComputer::new();
        fc.reset(); // should not panic
    }

    #[test]
    fn test_flight_computer_set_algorithm() {
        let mut fc = FlightComputer::new();
        assert_eq!(fc.config.algorithm, "cosine");
        // 切到自己（余弦）
        fc.set_algorithm("cosine");
        assert_eq!(fc.config.algorithm, "cosine");
    }

    fn dummy_state() -> GuidanceState {
        GuidanceState {
            altitude: 500.0,
            velocity_mag: 0.0,
            position: Vec3::new(0.0, 6_371_000.0, 0.0),
            velocity: Vec3::zero(),
            mission_time: 0.0,
            total_mass_kg: 1_000_000.0,
            stage: 0,
            throttle: 1.0,
        }
    }
}
