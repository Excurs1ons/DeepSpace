//! 空气动力学模块
//!
//! 提供大气飞行器/导弹的完整气动力模型：
//! - 标准大气延伸（100km+）
//! - 升力/阻力/侧力系数（依 Mach、AoA、侧滑角查表）
//! - 控制面操纵力矩
//! - 发动机/进气道模型（推力随高度/Mach 变化）

use crate::Vec3;

// =====================================================================
// 风场模型
// =====================================================================

/// 三维风场配置
#[derive(Debug, Clone, Copy)]
pub struct WindField {
    /// 海平面风速 (m/s)
    pub surface_speed: f64,
    /// 海平面风向 (deg, 0=北风, 90=东风, 270=西风)
    pub surface_dir_deg: f64,
    /// 边界层顶风速 (m/s)，~1000m 以上
    pub gradient_speed: f64,
    /// 边界层顶风向 (deg)
    pub gradient_dir_deg: f64,
    /// 边界层厚度 (m)，默认 ~600m
    pub boundary_layer_m: f64,
}

impl Default for WindField {
    fn default() -> Self {
        Self {
            surface_speed: 5.0,
            surface_dir_deg: 270.0,  // 西风
            gradient_speed: 25.0,
            gradient_dir_deg: 270.0, // 高空西风急流
            boundary_layer_m: 600.0,
        }
    }
}

impl WindField {
    /// 静风
    pub fn calm() -> Self {
        Self {
            surface_speed: 0.0,
            ..Default::default()
        }
    }

    /// 获取某高度风速向量 (m/s, 世界坐标系 ENU: x=东, y=北, z=上)
    pub fn wind_at(&self, altitude_m: f64) -> Vec3 {
        let h = altitude_m.max(0.0);

        // 边界层内用对数律插值，之上用梯度风
        let (frac, sfc_wt, grad_wt) = if h < self.boundary_layer_m {
            let f = (h / self.boundary_layer_m).sqrt().min(1.0);
            (h / self.boundary_layer_m, 1.0 - f, f)
        } else {
            (1.0, 0.0, 1.0)
        };

        // 合成风速和风向
        let speed = self.surface_speed * (1.0 - grad_wt) + self.gradient_speed * grad_wt;
        let dir_deg = self.surface_dir_deg * (1.0 - grad_wt) + self.gradient_dir_deg * grad_wt;

        // 风向 → ENU 分量 (x=东, y=北)
        // 0°=北风(向南吹) = -y, 90°=东风(向西吹) = -x
        // 风向是"来自"方向，风矢量是"去向"
        let dir_rad = dir_deg.to_radians();
        let u = -speed * dir_rad.sin(); // 东西分量 (+ = 向东)
        let v = -speed * dir_rad.cos(); // 南北分量 (+ = 向北)

        Vec3::new(u, v, 0.0)
    }
}

/// 默认高空风场实例
pub fn default_wind() -> WindField {
    WindField::default()
}

/// 获取某高度的大气密度、声速、温度
pub fn atmosphere_at(altitude_m: f64) -> AtmoState {
    let h = altitude_m.max(0.0);
    // 复用已有的 ISA 分层数据，外层用指数衰减延伸
    let (p, t) = if h <= 84_852.0 {
        earth_isa_below_85km(h)
    } else {
        earth_isa_above_85km(h)
    };
    let rho = if t > 0.0 { p / (287.058 * t) } else { 0.0 };
    let sos = if t > 0.0 {
        (1.4 * 287.058 * t).sqrt()
    } else {
        0.0
    };
    AtmoState { rho, p, t, sos }
}

#[derive(Debug, Clone, Copy)]
pub struct AtmoState {
    pub rho: f64,   // 密度 kg/m³
    pub p: f64,     // 压力 Pa
    pub t: f64,     // 温度 K
    pub sos: f64,   // 声速 m/s
}

fn earth_isa_below_85km(h: f64) -> (f64, f64) {
    const LAYERS: [(f64, f64, f64, f64, f64); 7] = [
        (0.0, 11000.0, 288.15, 101325.0, -0.0065),
        (11000.0, 20000.0, 216.65, 22632.06, 0.0),
        (20000.0, 32000.0, 216.65, 5474.889, 0.001),
        (32000.0, 47000.0, 228.65, 868.0187, 0.0028),
        (47000.0, 51000.0, 270.65, 110.9063, 0.0),
        (51000.0, 71000.0, 270.65, 66.93887, -0.0028),
        (71000.0, 84852.0, 214.65, 3.956420, -0.002),
    ];
    const EXP_SCALE: f64 = 0.034171; // G0*M/R
    for &(hb, ht, tb, pb, lapse) in LAYERS.iter() {
        if h <= ht {
            let dh = h - hb;
            if lapse.abs() < 1e-12 {
                return (pb * (-(EXP_SCALE * dh) / tb).exp(), tb);
            }
            let temp = tb + lapse * dh;
            let press = pb * (tb / temp).powf(EXP_SCALE / lapse);
            return (press, temp);
        }
    }
    (0.3734, 186.946) // fallback
}

fn earth_isa_above_85km(h: f64) -> (f64, f64) {
    // 84.852-100km: 等温过渡
    // 100km+: 指数衰减，scale_height ≈ 7000m
    const H_TOP: f64 = 84852.0;
    const P_TOP: f64 = 0.3734;
    const T_TOP: f64 = 186.946;
    let dh = (h - H_TOP).max(0.0);
    let p = P_TOP * (-dh / 7000.0).exp();
    (p, T_TOP)
}

// =====================================================================
// 气动系数
// =====================================================================

/// 飞行状态参数（气动计算输入）
#[derive(Debug, Clone, Copy)]
pub struct AeroState {
    pub alpha: f64,   // 攻角 (rad)
    pub beta: f64,    // 侧滑角 (rad)
    pub mach: f64,    // 马赫数
    pub alt: f64,     // 高度 (m)
    pub qbar: f64,    // 动压 (Pa)
}

impl AeroState {
    pub fn new(vel_body: Vec3, atmo: &AtmoState) -> Self {
        let speed = vel_body.length().max(1e-6);
        let alpha = (vel_body.z / speed).asin(); // 攻角: local z-up
        let beta = (vel_body.x / speed).asin();   // 侧滑角
        let mach = speed / atmo.sos.max(1.0);
        let qbar = 0.5 * atmo.rho * speed * speed;
        AeroState { alpha, beta, mach, alt: 0.0, qbar }
    }
}

/// 气动系数集
#[derive(Debug, Clone)]
pub struct AeroCoeffs {
    /// 零升阻力 Cd0
    pub cd0: f64,
    /// 升致阻力因子 K: Cd = Cd0 + K*CL²
    pub k_factor: f64,
    /// 升力线斜率 CL_alpha (per rad)
    pub cl_alpha: f64,
    /// 最大升力系数 (失速限制)
    pub cl_max: f64,
    /// 铰链力矩系数
    pub cm_alpha: f64,  // 纵向静稳导数
    pub cm_de: f64,     // 升降舵效率 (per rad)
    pub cn_beta: f64,   // 航向静稳导数
    pub cl_da: f64,     // 副翼效率 (per rad)
    pub cn_dr: f64,     // 方向舵效率 (per rad)
    // 阻力板
    pub cd_flap_delta: f64,
}

impl AeroCoeffs {
    /// 典型战斗机（F-16 级）气动数据
    pub fn fighter_typical() -> Self {
        AeroCoeffs {
            cd0: 0.025,
            k_factor: 0.12,
            cl_alpha: 5.5,  // per rad (~0.096/deg)
            cl_max: 1.6,
            cm_alpha: -0.5, // 负值 = 静稳定
            cm_de: -1.2,    // 升降舵下偏 → 抬头
            cn_beta: 0.12,
            cl_da: 0.15,    // 副翼差动
            cn_dr: -0.08,
            cd_flap_delta: 0.02,
        }
    }

    /// 典型导弹（AIM-120 级）气动数据
    pub fn missile_typical() -> Self {
        AeroCoeffs {
            cd0: 0.08,
            k_factor: 0.3,
            cl_alpha: 8.0,  // 导弹弹体升力效率高（翼面+弹体）
            cl_max: 2.5,
            cm_alpha: -1.0,
            cm_de: -2.0,    // 尾舵效率高
            cn_beta: 0.2,
            cl_da: 0.3,
            cn_dr: -0.15,
            cd_flap_delta: 0.01,
        }
    }

    /// 典型的弹道导弹再入体
    pub fn reentry_vehicle() -> Self {
        AeroCoeffs {
            cd0: 0.15,
            k_factor: 0.05,
            cl_alpha: 1.5,
            cl_max: 0.5,
            cm_alpha: -0.1,
            cm_de: 0.0,
            cn_beta: 0.02,
            cl_da: 0.0,
            cn_dr: 0.0,
            cd_flap_delta: 0.0,
        }
    }
}

// =====================================================================
// 气动力计算
// =====================================================================

/// 计算气动力（机体坐标系: x-前, y-右, z-下）
pub fn aero_forces(
    aero: &AeroState,
    coeffs: &AeroCoeffs,
    wing_area: f64,
    deflections: &ControlDeflections,
) -> Vec3 {
    let qs = aero.qbar * wing_area;
    let alpha = aero.alpha;
    let beta = aero.beta;

    // CL (考虑失速修正)
    let cl_linear = coeffs.cl_alpha * alpha;
    let cl_stall = stall_correction(cl_linear, coeffs.cl_max);
    // CD = Cd0 + K*CL² + 襟翼/减速板增量
    let cd = coeffs.cd0 + coeffs.k_factor * cl_stall * cl_stall
        + if deflections.speedbrake > 0.5 { coeffs.cd_flap_delta } else { 0.0 };
    // 侧力
    let cy = coeffs.cn_beta * beta; // 简化

    // 机体坐标: x-阻力, y-侧力, z-升力（负值=向上）
    Vec3::new(
        -qs * cd,
        qs * cy,
        -qs * cl_stall,
    )
}

/// 失速修正：在 CL_max 附近圆滑饱和
fn stall_correction(cl: f64, cl_max: f64) -> f64 {
    let abs_cl = cl.abs();
    if abs_cl <= cl_max * 0.8 {
        cl
    } else if abs_cl <= cl_max {
        cl.signum() * (cl_max * 0.8 + (abs_cl - cl_max * 0.8) * 0.3)
    } else {
        cl.signum() * (cl_max * 0.8 + (cl_max - cl_max * 0.8) * 0.3
            + (abs_cl - cl_max) * 0.05)
    }
}

/// 气动力矩（body-frame）
pub fn aero_moments(
    aero: &AeroState,
    coeffs: &AeroCoeffs,
    wing_area: f64,
    span: f64,
    chord: f64,
    deflections: &ControlDeflections,
) -> Vec3 {
    let qs = aero.qbar * wing_area;
    let alpha = aero.alpha;
    let beta = aero.beta;

    // 俯仰力矩 (y-axis): Cm = Cm_alpha * alpha + Cm_de * elevator
    let cm = coeffs.cm_alpha * alpha + coeffs.cm_de * deflections.elevator;
    let pitch_moment = qs * chord * cm;

    // 滚转力矩 (x-axis): Cl = Cl_da * aileron（简化无上反角效应）
    let cl_roll = coeffs.cl_da * deflections.aileron;
    let roll_moment = qs * span * cl_roll;

    // 偏航力矩 (z-axis): Cn = Cn_beta * beta + Cn_dr * rudder
    let cn = coeffs.cn_beta * beta + coeffs.cn_dr * deflections.rudder;
    let yaw_moment = qs * span * cn;

    Vec3::new(roll_moment, pitch_moment, yaw_moment)
}

/// 控制面偏转（弧度）
#[derive(Debug, Clone, Copy)]
pub struct ControlDeflections {
    pub elevator: f64,   // 升降舵 (+下偏=抬头)
    pub aileron: f64,    // 副翼 (+右滚)
    pub rudder: f64,     // 方向舵 (+右偏航)
    pub speedbrake: f64, // 减速板 (0~1)
    pub throttle: f64,   // 油门 (0~1)
}

impl ControlDeflections {
    pub fn neutral() -> Self {
        ControlDeflections {
            elevator: 0.0, aileron: 0.0, rudder: 0.0,
            speedbrake: 0.0, throttle: 0.0,
        }
    }
}

// =====================================================================
// 推力模型
// =====================================================================

/// 计算喷气发动机推力（高度/Mach 修正）
pub fn jet_thrust(
    sea_level_static_thrust: f64,
    mach: f64,
    altitude: f64,
    throttle: f64,
) -> f64 {
    if throttle <= 0.0 || sea_level_static_thrust <= 0.0 {
        return 0.0;
    }
    // 高度修正: 密度比
    let rho_sl = 1.225;
    let rho = if altitude <= 84_852.0 {
        let (p, t) = earth_isa_below_85km(altitude);
        if t > 0.0 { p / (287.058 * t) } else { 0.0 }
    } else {
        let (p, t) = earth_isa_above_85km(altitude);
        if t > 0.0 { p / (287.058 * t) } else { 0.0 }
    };
    let alt_factor = (rho / rho_sl).max(0.0);

    // Mach 修正: 亚音速近似常数，超音速略有变化
    let mach_factor = if mach <= 1.0 {
        1.0
    } else if mach <= 2.5 {
        1.0 - (mach - 1.0) * 0.08
    } else {
        0.88
    };

    sea_level_static_thrust * throttle * alt_factor * mach_factor
}

/// 火箭发动机推力（真空 + 高度修正）
pub fn rocket_thrust(
    vac_thrust: f64,
    ambient_pressure: f64,
    exit_area: f64,
    throttle: f64,
) -> f64 {
    if throttle <= 0.0 {
        return 0.0;
    }
    // F = F_vac - p_ambient * A_exit
    let thrust = vac_thrust - ambient_pressure * exit_area;
    thrust.max(0.0) * throttle
}

// =====================================================================
// 单位测试
// =====================================================================
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn atmosphere_sea_level() {
        let s = atmosphere_at(0.0);
        assert!((s.rho - 1.225).abs() < 0.02);
        assert!((s.p - 101325.0).abs() < 200.0);
        assert!((s.sos - 340.0).abs() < 5.0);
    }

    #[test]
    fn atmosphere_high_alt() {
        let s = atmosphere_at(100_000.0);
        assert!(s.rho > 0.0 && s.rho < 1e-4);
        assert!(s.p > 0.0 && s.p < 1.0);
        assert!(s.t > 180.0);
    }

    #[test]
    fn aero_forces_zero_alpha() {
        let state = AeroState {
            alpha: 0.0, beta: 0.0, mach: 0.5, alt: 0.0, qbar: 1000.0,
        };
        let coeffs = AeroCoeffs::fighter_typical();
        let defs = ControlDeflections::neutral();
        let f = aero_forces(&state, &coeffs, 30.0, &defs);
        // 零攻角应只有阻力
        assert!(f.x < 0.0);
        assert!((f.y).abs() < 1e-6);
        assert!((f.z).abs() < 1e-6);
    }

    #[test]
    fn aero_forces_lift_at_alpha() {
        let state = AeroState {
            alpha: 0.1, beta: 0.0, mach: 0.5, alt: 5000.0, qbar: 2000.0,
        };
        let coeffs = AeroCoeffs::fighter_typical();
        let defs = ControlDeflections::neutral();
        let f = aero_forces(&state, &coeffs, 30.0, &defs);
        // 正攻角产生负 Z 升力（向上）
        assert!(f.z < -1.0);
    }

    #[test]
    fn aero_moments_pitch_up() {
        let state = AeroState {
            alpha: 0.05, beta: 0.0, mach: 0.6, alt: 0.0, qbar: 5000.0,
        };
        let coeffs = AeroCoeffs::fighter_typical();
        let mut defs = ControlDeflections::neutral();
        defs.elevator = -0.1; // 抬升降舵 → 抬头（正俯仰力矩）
        let m = aero_moments(&state, &coeffs, 30.0, 10.0, 4.0, &defs);
        assert!(m.y > 0.0); // 正俯仰力矩 = 抬头
    }

    #[test]
    fn jet_thrust_at_altitude() {
        let t = jet_thrust(100_000.0, 0.8, 0.0, 1.0);
        assert!((t - 100_000.0).abs() < 2000.0);
        let t_high = jet_thrust(100_000.0, 0.8, 10_000.0, 1.0);
        assert!(t_high > 0.0 && t_high < t); // 高空推力下降
    }

    #[test]
    fn jet_thrust_zero_throttle() {
        let t = jet_thrust(100_000.0, 0.8, 0.0, 0.0);
        assert!((t).abs() < 0.01);
    }

    #[test]
    fn rocket_thrust_vac() {
        let t = rocket_thrust(500_000.0, 0.0, 1.0, 1.0);
        assert!((t - 500_000.0).abs() < 1.0);
        let t_sl = rocket_thrust(500_000.0, 101325.0, 1.0, 1.0);
        assert!(t_sl < t); // 地面推力小于真空
    }

    #[test]
    fn stall_correction_clamp() {
        let cl = stall_correction(2.0, 1.6);
        assert!(cl < 2.0); // 饱和
        assert!(cl > 1.0); // 仍有一定升力
    }

    #[test]
    fn aero_state_from_velocity() {
        let atmo = atmosphere_at(0.0);
        let vel = Vec3::new(0.0, 0.0, -300.0); // 向下飞
        let s = AeroState::new(vel, &atmo);
        assert!(s.alpha < 0.0); // 负攻角
        assert!((s.mach - 300.0 / atmo.sos).abs() < 0.01);
        assert!(s.qbar > 0.0);
    }
}
