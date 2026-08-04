//! 飞行器平台模块
//!
//! 定义战斗机、轰炸机、无人机等大气层内飞行器的配置和动力学。
//! 结合 aerodynamics 模块的气动力 + 重力 + 推力进行 6DOF 积分。

use crate::aerodynamics::{
    aero_forces, aero_moments, jet_thrust, AeroCoeffs, AeroState, AtmoState, ControlDeflections,
    WindField,
};
use crate::core::Quaternion;
use crate::Vec3;

// =====================================================================
// 飞行器配置
// =====================================================================

/// 飞机类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AircraftCategory {
    Fighter,
    Bomber,
    Awacs,
    Tanker,
    Uav,
    Trainer,
}

/// 飞行器配置
#[derive(Debug, Clone)]
pub struct AircraftConfig {
    pub name: String,
    pub category: AircraftCategory,

    // 几何
    pub mass_empty_kg: f64,
    pub max_fuel_kg: f64,
    pub wing_area_m2: f64,
    pub wing_span_m: f64,
    pub mac_m: f64, // mean aerodynamic chord

    // 发动机
    pub engine_count: i32,
    pub engine_thrust_n: f64, // 单台海平面静推力

    // 气动
    pub aero_coeffs: AeroCoeffs,

    // 限制
    pub max_mach: f64,
    pub max_g_pos: f64,
    pub max_g_neg: f64,
    pub max_altitude_m: f64,
    pub max_aoa_deg: f64,

    // 雷达截面积
    pub rcs_m2: f64,

    // 最大军械载荷
    pub max_payload_kg: f64,
}

impl AircraftConfig {
    pub fn f22() -> Self {
        AircraftConfig {
            name: "F-22 Raptor".into(),
            category: AircraftCategory::Fighter,
            mass_empty_kg: 19_700.0,
            max_fuel_kg: 9_100.0,
            wing_area_m2: 78.0,
            wing_span_m: 13.56,
            mac_m: 5.0,
            engine_count: 2,
            engine_thrust_n: 156_000.0, // F119-PW-100 approx
            aero_coeffs: AeroCoeffs::fighter_typical(),
            max_mach: 2.25,
            max_g_pos: 9.0,
            max_g_neg: -3.0,
            max_altitude_m: 20_000.0,
            max_aoa_deg: 60.0,
            rcs_m2: 0.0001, // stealth
            max_payload_kg: 2_270.0,
        }
    }

    pub fn f16() -> Self {
        AircraftConfig {
            name: "F-16 Fighting Falcon".into(),
            category: AircraftCategory::Fighter,
            mass_empty_kg: 8_570.0,
            max_fuel_kg: 3_200.0,
            wing_area_m2: 27.87,
            wing_span_m: 9.96,
            mac_m: 3.5,
            engine_count: 1,
            engine_thrust_n: 131_000.0, // F110-GE-129
            aero_coeffs: AeroCoeffs::fighter_typical(),
            max_mach: 2.05,
            max_g_pos: 9.0,
            max_g_neg: -3.0,
            max_altitude_m: 15_240.0,
            max_aoa_deg: 40.0,
            rcs_m2: 5.0, // non-stealth
            max_payload_kg: 7_800.0,
        }
    }

    pub fn su57() -> Self {
        AircraftConfig {
            name: "Su-57 Felon".into(),
            category: AircraftCategory::Fighter,
            mass_empty_kg: 18_000.0,
            max_fuel_kg: 10_300.0,
            wing_area_m2: 82.0,
            wing_span_m: 14.1,
            mac_m: 5.2,
            engine_count: 2,
            engine_thrust_n: 176_000.0, // Izd-30 approx
            aero_coeffs: AeroCoeffs::fighter_typical(),
            max_mach: 2.0,
            max_g_pos: 9.0,
            max_g_neg: -3.0,
            max_altitude_m: 20_000.0,
            max_aoa_deg: 60.0,
            rcs_m2: 0.001,
            max_payload_kg: 7_500.0,
        }
    }

    pub fn su35() -> Self {
        AircraftConfig {
            name: "Su-35 Flanker-E".into(),
            category: AircraftCategory::Fighter,
            mass_empty_kg: 16_500.0,
            max_fuel_kg: 11_300.0,
            wing_area_m2: 62.0,
            wing_span_m: 15.3,
            mac_m: 5.0,
            engine_count: 2,
            engine_thrust_n: 142_000.0,
            aero_coeffs: AeroCoeffs::fighter_typical(),
            max_mach: 2.25,
            max_g_pos: 9.0,
            max_g_neg: -3.0,
            max_altitude_m: 18_000.0,
            max_aoa_deg: 45.0,
            rcs_m2: 3.0,
            max_payload_kg: 8_000.0,
        }
    }

    pub fn j20() -> Self {
        AircraftConfig {
            name: "J-20 Mighty Dragon".into(),
            category: AircraftCategory::Fighter,
            mass_empty_kg: 19_400.0,
            max_fuel_kg: 9_500.0,
            wing_area_m2: 75.0,
            wing_span_m: 13.0,
            mac_m: 5.0,
            engine_count: 2,
            engine_thrust_n: 156_000.0,
            aero_coeffs: AeroCoeffs::fighter_typical(),
            max_mach: 2.0,
            max_g_pos: 9.0,
            max_g_neg: -3.0,
            max_altitude_m: 20_000.0,
            max_aoa_deg: 55.0,
            rcs_m2: 0.0005,
            max_payload_kg: 8_000.0,
        }
    }

    pub fn b52() -> Self {
        AircraftConfig {
            name: "B-52 Stratofortress".into(),
            category: AircraftCategory::Bomber,
            mass_empty_kg: 83_250.0,
            max_fuel_kg: 135_000.0,
            wing_area_m2: 370.0,
            wing_span_m: 56.4,
            mac_m: 8.0,
            engine_count: 8,
            engine_thrust_n: 76_000.0,
            aero_coeffs: AeroCoeffs::fighter_typical(),
            max_mach: 0.9,
            max_g_pos: 3.0,
            max_g_neg: -1.5,
            max_altitude_m: 15_000.0,
            max_aoa_deg: 20.0,
            rcs_m2: 100.0, // huge radar signature
            max_payload_kg: 31_500.0,
        }
    }

    /// 总质量（空重 + 剩余燃油）
    pub fn total_mass(&self, fuel_kg: f64) -> f64 {
        self.mass_empty_kg + fuel_kg.min(self.max_fuel_kg)
    }
}

// =====================================================================
// 飞行器状态
// =====================================================================

/// 飞行器完整状态（6DOF）
#[derive(Debug, Clone)]
pub struct AircraftState {
    pub config: AircraftConfig,

    // 位置和速度（世界坐标 NEU: x=North, y=East, z=Up）
    pub position: Vec3,
    pub velocity: Vec3,

    // 姿态
    pub orientation: Quaternion,
    pub angular_velocity: Vec3,

    // 状态
    pub fuel_kg: f64,
    pub total_mass_kg: f64,

    // 控制面
    pub controls: ControlDeflections,

    // 当前气动状态（缓存）
    pub aero: AeroState,
    pub alpha_deg: f64,
    pub beta_deg: f64,
    pub mach: f64,
    pub altitude_m: f64,
    pub g_load: f64,
    pub dynamic_pressure: f64,

    // 累加力和力矩
    acc_force_world: Vec3,
    acc_torque_body: Vec3,
}

impl AircraftState {
    pub fn new(
        config: AircraftConfig,
        pos: Vec3,
        vel: Vec3,
        heading_deg: f64,
        fuel_kg: f64,
    ) -> Self {
        let fuel = fuel_kg.min(config.max_fuel_kg);
        let mass = config.total_mass(fuel);
        // 按航向初始化姿态（z-up: heading为真北顺时针）
        let heading_rad = heading_deg.to_radians();
        let orientation = Quaternion::from_euler(0.0, heading_rad, 0.0);

        AircraftState {
            aero: AeroState {
                alpha: 0.0,
                beta: 0.0,
                mach: 0.0,
                alt: 0.0,
                qbar: 0.0,
            },
            alpha_deg: 0.0,
            beta_deg: 0.0,
            mach: 0.0,
            altitude_m: pos.z,
            g_load: 1.0,
            dynamic_pressure: 0.0,
            position: pos,
            velocity: vel,
            orientation,
            angular_velocity: Vec3::zero(),
            fuel_kg: fuel,
            total_mass_kg: mass,
            controls: ControlDeflections::neutral(),
            config,
            acc_force_world: Vec3::zero(),
            acc_torque_body: Vec3::zero(),
        }
    }

    pub fn add_force(&mut self, force_world: Vec3) {
        self.acc_force_world += force_world;
    }

    pub fn add_torque(&mut self, torque_body: Vec3) {
        self.acc_torque_body += torque_body;
    }

    // ---- 前向更新 ----
    /// 推进一个物理步
    pub fn step(&mut self, dt: f64, atmo: &AtmoState) {
        if self.total_mass_kg <= 0.0 || dt <= 0.0 {
            return;
        }

        // 1. 计算气动状态（空速 = 地速 − 风速）
        let vel_body = self.orientation.conjugate().rotate(&self.velocity);
        let wind_world = WindField::default().wind_at(self.position.z);
        let wind_body = self.orientation.conjugate().rotate(&wind_world);
        let air_vel_body = vel_body - wind_body;
        let aero = AeroState::new(air_vel_body, atmo);
        let qbar = aero.qbar;
        let alpha_deg = aero.alpha.to_degrees();
        let beta_deg = aero.beta.to_degrees();
        let mach = aero.mach;
        self.aero = aero;
        self.alpha_deg = alpha_deg;
        self.beta_deg = beta_deg;
        self.mach = mach;
        self.dynamic_pressure = qbar;
        self.altitude_m = self.position.z;

        let m = self.total_mass_kg;
        let w = self.config.wing_area_m2;
        let b = self.config.wing_span_m;
        let c = self.config.mac_m;
        let coeffs = &self.config.aero_coeffs;

        // 2. 气动力（body-frame）→ 转 world
        let force_body = aero_forces(&self.aero, coeffs, w, &self.controls);
        let force_world = self.orientation.rotate(&force_body);
        self.acc_force_world += force_world;

        // 3. 气动力矩
        let torque_body = aero_moments(&self.aero, coeffs, w, b, c, &self.controls);
        self.acc_torque_body += torque_body;

        // 4. 推力
        let thrust = jet_thrust(
            self.config.engine_thrust_n * self.config.engine_count as f64,
            self.aero.mach,
            self.altitude_m,
            self.controls.throttle,
        );
        // 推力沿机体 x 方向（前）
        let thrust_body = Vec3::new(thrust, 0.0, 0.0);
        let thrust_world = self.orientation.rotate(&thrust_body);
        self.acc_force_world += thrust_world;

        // 5. 重力
        self.acc_force_world += Vec3::new(0.0, 0.0, -m * 9.80665);

        // 6. 积分（半隐式欧拉）
        let accel = self.acc_force_world / m;
        self.velocity += accel * dt;
        self.position += self.velocity * dt;

        // 角速度积分
        let inertia = m * (b * b + c * c) / 12.0; // 简化的惯性矩
        if inertia > 0.0 {
            let ang_accel = self.acc_torque_body / inertia;
            self.angular_velocity += ang_accel * dt;
        }

        // 四元数更新
        let wq = Quaternion::new(
            0.0,
            self.angular_velocity.x,
            self.angular_velocity.y,
            self.angular_velocity.z,
        );
        let dq = wq.mul(&self.orientation);
        self.orientation = Quaternion::new(
            self.orientation.w + 0.5 * dt * dq.w,
            self.orientation.x + 0.5 * dt * dq.x,
            self.orientation.y + 0.5 * dt * dq.y,
            self.orientation.z + 0.5 * dt * dq.z,
        )
        .normalized();

        // 7. g-load
        let total_accel_mag = accel.length() / 9.80665;
        self.g_load = total_accel_mag;

        // 8. 燃油消耗
        if self.controls.throttle > 0.01 {
            let sfc = 0.8 / 3600.0; // 简化 SFC: 每牛每小时 0.8 kg
            let fuel_burn = thrust * sfc * dt;
            self.fuel_kg = (self.fuel_kg - fuel_burn).max(0.0);
            self.total_mass_kg = self.config.total_mass(self.fuel_kg);
        }

        // 重置累加器
        self.acc_force_world = Vec3::zero();
        self.acc_torque_body = Vec3::zero();
    }

    // ---- 辅助 ----

    pub fn speed(&self) -> f64 {
        self.velocity.length()
    }

    pub fn mach_number(&self) -> f64 {
        self.mach
    }

    pub fn get_position_geo(&self) -> (f64, f64, f64) {
        // 假设 NEU 坐标：x=北(N), y=东(E), z=天顶(Up)
        // 地球半径 R≈6371km，原点在 (lat0, lon0, alt0)
        // 这里返回相对于起点的偏移（用户可根据需要转换）
        (self.position.x, self.position.y, self.position.z)
    }
}

// =====================================================================
// 自动驾驶仪
// =====================================================================

/// 简易自动驾驶模式
#[derive(Debug, Clone, PartialEq)]
pub enum AutopilotMode {
    /// 保持航向、高度、速度
    Hold,
    /// 转向目标航向并保持
    TurnToHeading(f64),
    /// 爬升到目标高度
    ClimbTo(f64),
    /// 加速到目标马赫数
    MachHold(f64),
    /// 跟随航点
    Waypoint,
    /// 关闭
    Off,
}

/// 简易自动驾驶仪（PID 控制）
#[derive(Debug, Clone)]
pub struct Autopilot {
    pub mode: AutopilotMode,
    pub target_heading_deg: f64,
    pub target_altitude_m: f64,
    pub target_mach: f64,
    // PID state
    prev_heading_error: f64,
    prev_alt_error: f64,
    i_heading: f64,
    i_alt: f64,
}

impl Default for Autopilot {
    fn default() -> Self {
        Self::new()
    }
}

impl Autopilot {
    pub fn new() -> Self {
        Autopilot {
            mode: AutopilotMode::Off,
            target_heading_deg: 0.0,
            target_altitude_m: 10_000.0,
            target_mach: 0.8,
            prev_heading_error: 0.0,
            prev_alt_error: 0.0,
            i_heading: 0.0,
            i_alt: 0.0,
        }
    }

    /// 计算控制面偏转
    pub fn compute(&mut self, state: &AircraftState, dt: f64) -> ControlDeflections {
        let mut defs = ControlDeflections::neutral();

        if self.mode == AutopilotMode::Off {
            return defs;
        }

        let kp_h = 0.5;
        let kd_h = 0.1;
        let kp_a = 0.005;
        let kd_a = 0.001;
        let kp_throttle = 0.3;

        // 航向保持（PID）
        let current_heading = state.orientation.yaw_deg();
        let mut heading_error = self.target_heading_deg - current_heading;
        if heading_error > 180.0 {
            heading_error -= 360.0;
        }
        if heading_error < -180.0 {
            heading_error += 360.0;
        }

        self.i_heading += heading_error * dt * 0.02;
        self.i_heading = self.i_heading.clamp(-0.5, 0.5);

        let d_heading = (heading_error - self.prev_heading_error) / dt.max(0.01);
        let roll_cmd = (kp_h * heading_error + kd_h * d_heading + self.i_heading).clamp(-1.0, 1.0);
        defs.aileron = roll_cmd * 0.3;
        // 协调转弯：偏航跟随
        defs.rudder = roll_cmd * 0.1;

        self.prev_heading_error = heading_error;

        // 高度保持（PID）
        let alt_error = self.target_altitude_m - state.altitude_m;
        self.i_alt += alt_error * dt * 0.005;
        self.i_alt = self.i_alt.clamp(-0.3, 0.3);
        let d_alt = (alt_error - self.prev_alt_error) / dt.max(0.01);
        let pitch_cmd = (kp_a * alt_error + kd_a * d_alt + self.i_alt).clamp(-0.5, 0.5);
        defs.elevator = pitch_cmd;

        self.prev_alt_error = alt_error;

        // 油门控制（保持目标 Mach）
        let mach_error = self.target_mach - state.mach;
        defs.throttle = (0.5 + kp_throttle * mach_error).clamp(0.0, 1.0);

        defs
    }
}

// =====================================================================
// 测试
// =====================================================================
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aircraft_f16_creation() {
        let f16 = AircraftConfig::f16();
        let mass = f16.total_mass(2000.0);
        assert!(mass > 8_000.0 && mass < 12_000.0);
        assert!(f16.max_mach > 2.0);
    }

    #[test]
    fn aircraft_step_straight_level() {
        let cfg = AircraftConfig::f16();
        let mut ac = AircraftState::new(
            cfg,
            Vec3::new(0.0, 0.0, 5000.0),
            Vec3::new(250.0, 0.0, 0.0),
            0.0,
            2000.0,
        );
        ac.controls.throttle = 0.5;

        let atmo = crate::aerodynamics::atmosphere_at(5000.0);
        ac.step(0.01, &atmo);

        // 应保持大致速度
        assert!(ac.speed() > 100.0 && ac.speed() < 400.0);
        assert!(ac.total_mass_kg > 0.0);
    }

    #[test]
    fn autopilot_heading_hold() {
        let mut ap = Autopilot::new();
        ap.mode = AutopilotMode::Hold;
        ap.target_heading_deg = 90.0;

        let cfg = AircraftConfig::f16();
        let ac = AircraftState::new(
            cfg,
            Vec3::zero(),
            Vec3::new(250.0, 0.0, 5000.0),
            0.0,
            2000.0,
        );

        let _defs = ap.compute(&ac, 0.05);
        // 至少应有非零控制
    }

    #[test]
    fn aircraft_categories() {
        let f22 = AircraftConfig::f22();
        let su35 = AircraftConfig::su35();
        let j20 = AircraftConfig::j20();
        let b52 = AircraftConfig::b52();
        assert!(f22.rcs_m2 < su35.rcs_m2); // F-22 比 Su-35 隐身
        assert!(j20.rcs_m2 < 0.001);
        assert!(b52.max_payload_kg > 30_000.0);
    }
}
