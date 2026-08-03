use crate::Vec3;
use crate::{EARTH_EQUATORIAL_RADIUS, EARTH_MU, EARTH_OMEGA, EARTH_POLAR_RADIUS};
use std::f64::consts::PI;

// 地球椭球辅助函数
struct EarthModel;

impl EarthModel {
    fn radius_at_latitude(lat_deg: f64) -> f64 {
        let lat = lat_deg.to_radians();
        let a = EARTH_EQUATORIAL_RADIUS;
        let b = EARTH_POLAR_RADIUS;
        let a2 = a * a;
        let b2 = b * b;
        let cos_lat = lat.cos();
        let sin_lat = lat.sin();
        ((a2 * cos_lat).powi(2) + (b2 * sin_lat).powi(2)).sqrt()
            / ((a * cos_lat).powi(2) + (b * sin_lat).powi(2)).sqrt()
    }

    fn coriolis_accel(vel: Vec3) -> Vec3 {
        // -2 ω × v
        Vec3 {
            x: 2.0 * EARTH_OMEGA * vel.y,
            y: -2.0 * EARTH_OMEGA * vel.x,
            z: 0.0,
        }
    }
}

// =====================================================================
// 级配置
// =====================================================================
#[derive(Debug, Clone)]
pub struct StageConfig {
    pub name: String,
    pub propellant_mass_kg: f64,
    pub structural_mass_kg: f64,
    pub thrust_vac_n: f64,
    pub thrust_sl_n: f64,
    pub isp_sl_s: f64,  // 海平面比冲 (s)
    pub isp_vac_s: f64, // 真空比冲 (s)
    pub burn_duration_s: f64,
    pub exit_area_m2: f64, // 喷管出口面积
}

// =====================================================================
// 再入飞行器
// =====================================================================
#[derive(Debug, Clone)]
pub struct ReentryVehicle {
    pub name: String,
    pub mass_kg: f64,
    pub cd: f64,
    pub nose_radius_m: f64,
    pub cross_section_m2: f64,
}

// =====================================================================
// ICBM 配置
// =====================================================================
#[derive(Debug, Clone)]
pub struct IcbmConfig {
    pub name: String,
    pub stages: Vec<StageConfig>,
    pub reentry_vehicles: Vec<ReentryVehicle>,
    pub bus_mass_kg: f64, // MIRV 母舱质量
    /// 发射点（纬度、经度、高度）
    pub launch_lat_deg: f64,
    pub launch_lon_deg: f64,
    pub launch_alt_m: f64,
    /// 目标位置
    pub target_lat_deg: f64,
    pub target_lon_deg: f64,
}

impl IcbmConfig {
    pub fn minuteman3() -> Self {
        IcbmConfig {
            name: "LGM-30 Minuteman III".into(),
            stages: vec![
                StageConfig {
                    name: "Stage 1".into(),
                    propellant_mass_kg: 21_800.0,
                    structural_mass_kg: 1_900.0,
                    thrust_vac_n: 1_800_000.0,
                    thrust_sl_n: 1_550_000.0,
                    isp_sl_s: 265.0,
                    isp_vac_s: 290.0,
                    burn_duration_s: 60.0,
                    exit_area_m2: 1.5,
                },
                StageConfig {
                    name: "Stage 2".into(),
                    propellant_mass_kg: 6_600.0,
                    structural_mass_kg: 1_000.0,
                    thrust_vac_n: 440_000.0,
                    thrust_sl_n: 0.0,
                    isp_sl_s: 260.0,
                    isp_vac_s: 289.0,
                    burn_duration_s: 60.0,
                    exit_area_m2: 1.0,
                },
                StageConfig {
                    name: "Stage 3".into(),
                    propellant_mass_kg: 3_350.0,
                    structural_mass_kg: 700.0,
                    thrust_vac_n: 340_000.0,
                    thrust_sl_n: 0.0,
                    isp_sl_s: 0.0,
                    isp_vac_s: 295.0,
                    burn_duration_s: 60.0,
                    exit_area_m2: 0.5,
                },
            ],
            reentry_vehicles: vec![
                ReentryVehicle {
                    name: "Mk-21 RV".into(),
                    mass_kg: 200.0,
                    cd: 0.15,
                    nose_radius_m: 0.3,
                    cross_section_m2: 0.5,
                },
                ReentryVehicle {
                    name: "Mk-21 RV".into(),
                    mass_kg: 200.0,
                    cd: 0.15,
                    nose_radius_m: 0.3,
                    cross_section_m2: 0.5,
                },
                ReentryVehicle {
                    name: "Mk-21 RV".into(),
                    mass_kg: 200.0,
                    cd: 0.15,
                    nose_radius_m: 0.3,
                    cross_section_m2: 0.5,
                },
            ],
            bus_mass_kg: 500.0,
            launch_lat_deg: 48.0,
            launch_lon_deg: -110.0,
            launch_alt_m: 500.0,
            target_lat_deg: 41.0,
            target_lon_deg: -81.0, // ~2400 km（导弹自然落点）
        }
    }

    pub fn df41() -> Self {
        IcbmConfig {
            name: "DF-41".into(),
            stages: vec![
                StageConfig {
                    name: "Stage 1".into(),
                    propellant_mass_kg: 30_000.0,
                    structural_mass_kg: 3_000.0,
                    thrust_vac_n: 3_000_000.0,
                    thrust_sl_n: 2_600_000.0,
                    isp_sl_s: 270.0,
                    isp_vac_s: 295.0,
                    burn_duration_s: 65.0,
                    exit_area_m2: 2.5,
                },
                StageConfig {
                    name: "Stage 2".into(),
                    propellant_mass_kg: 8_000.0,
                    structural_mass_kg: 1_000.0,
                    thrust_vac_n: 1_000_000.0,
                    thrust_sl_n: 0.0,
                    isp_sl_s: 260.0,
                    isp_vac_s: 300.0,
                    burn_duration_s: 60.0,
                    exit_area_m2: 1.2,
                },
                StageConfig {
                    name: "Stage 3".into(),
                    propellant_mass_kg: 2_500.0,
                    structural_mass_kg: 400.0,
                    thrust_vac_n: 400_000.0,
                    thrust_sl_n: 0.0,
                    isp_sl_s: 0.0,
                    isp_vac_s: 305.0,
                    burn_duration_s: 45.0,
                    exit_area_m2: 0.6,
                },
            ],
            reentry_vehicles: vec![
                ReentryVehicle {
                    name: "RV".into(),
                    mass_kg: 300.0,
                    cd: 0.12,
                    nose_radius_m: 0.35,
                    cross_section_m2: 0.4,
                },
                ReentryVehicle {
                    name: "RV".into(),
                    mass_kg: 300.0,
                    cd: 0.12,
                    nose_radius_m: 0.35,
                    cross_section_m2: 0.4,
                },
                ReentryVehicle {
                    name: "RV".into(),
                    mass_kg: 300.0,
                    cd: 0.12,
                    nose_radius_m: 0.35,
                    cross_section_m2: 0.4,
                },
                ReentryVehicle {
                    name: "RV".into(),
                    mass_kg: 300.0,
                    cd: 0.12,
                    nose_radius_m: 0.35,
                    cross_section_m2: 0.4,
                },
                ReentryVehicle {
                    name: "RV".into(),
                    mass_kg: 300.0,
                    cd: 0.12,
                    nose_radius_m: 0.35,
                    cross_section_m2: 0.4,
                },
            ],
            bus_mass_kg: 800.0,
            launch_lat_deg: 40.0,
            launch_lon_deg: 116.0,
            launch_alt_m: 500.0,
            target_lat_deg: 48.0,
            target_lon_deg: -110.0,
        }
    }

    pub fn total_propellant_mass(&self) -> f64 {
        self.stages.iter().map(|s| s.propellant_mass_kg).sum()
    }

    pub fn total_mass(&self) -> f64 {
        let stage_mass: f64 = self
            .stages
            .iter()
            .map(|s| s.propellant_mass_kg + s.structural_mass_kg)
            .sum();
        let rv_mass: f64 = self.reentry_vehicles.iter().map(|rv| rv.mass_kg).sum();
        stage_mass + rv_mass + self.bus_mass_kg
    }
}

// =====================================================================
// 弹道阶段
// =====================================================================
#[derive(Debug, Clone, PartialEq)]
pub enum IcbmPhase {
    PreLaunch,
    Boost(usize), // 当前级索引
    Coast,
    Midcourse, // 中段飞行
    BusDeployment,
    Reentry,
    Impact,
    Failed,
}

// =====================================================================
// ICBM 飞行状态
// =====================================================================
#[derive(Debug, Clone)]
pub struct IcbmState {
    pub phase: IcbmPhase,
    pub position_ecef: Vec3,
    pub velocity_ecef: Vec3,
    pub heading_deg: f64,
    pub pitch_deg: f64,
    pub flight_time: f64,
    pub current_stage: usize,
    pub stage_time: f64,
    pub propellant_remaining: Vec<f64>,
    pub rv_deployed: Vec<bool>,
    pub apogee_m: f64,
    pub range_to_target_m: f64,
    pub config: IcbmConfig,
}

impl IcbmState {
    pub fn new(config: IcbmConfig) -> Self {
        let propellant = config.stages.iter().map(|s| s.propellant_mass_kg).collect();
        IcbmState {
            phase: IcbmPhase::PreLaunch,
            position_ecef: Vec3::zero(),
            velocity_ecef: Vec3::zero(),
            heading_deg: 0.0,
            pitch_deg: 90.0,
            flight_time: 0.0,
            current_stage: 0,
            stage_time: 0.0,
            propellant_remaining: propellant,
            rv_deployed: vec![false; config.reentry_vehicles.len()],
            apogee_m: 0.0,
            range_to_target_m: 0.0,
            config,
        }
    }

    pub fn launch(&mut self) {
        self.phase = IcbmPhase::Boost(0);
        self.current_stage = 0;
        self.stage_time = 0.0;
        self.flight_time = 0.0;
        self.init_position();
        // 计算发射方位角（指向目标的初始航向）
        self.heading_deg = self.bearing_to_target();
    }

    fn init_position(&mut self) {
        let lat = self.config.launch_lat_deg.to_radians();
        let lon = self.config.launch_lon_deg.to_radians();
        let r = EARTH_EQUATORIAL_RADIUS + self.config.launch_alt_m;
        self.position_ecef = Vec3::new(
            r * lat.cos() * lon.cos(),
            r * lat.cos() * lon.sin(),
            r * lat.sin(),
        );
        // 初始速度 = 地球自转速度
        self.velocity_ecef = Vec3::new(
            -EARTH_OMEGA * self.position_ecef.y,
            EARTH_OMEGA * self.position_ecef.x,
            0.0,
        );
    }

    /// 将 ECEF 坐标转地理坐标（椭球反算，Bowring 迭代法）
    ///
    /// 与 `EarthModel::radius_at_latitude` 使用的椭球模型严格互逆，
    /// 返回大地纬度、经度、椭球高度（真实海拔，非球面近似）。
    pub fn ecef_to_geo(pos: Vec3) -> (f64, f64, f64) {
        let a = EARTH_EQUATORIAL_RADIUS;
        let b = EARTH_POLAR_RADIUS;
        let e2 = 1.0 - (b * b) / (a * a); // 第一偏心率平方

        let lon = pos.y.atan2(pos.x);

        let p = (pos.x * pos.x + pos.y * pos.y).sqrt(); // 到极轴距离
        if p < 1e-7 {
            // 极点：lat = ±90°
            let lat = if pos.z >= 0.0 {
                std::f64::consts::FRAC_PI_2
            } else {
                -std::f64::consts::FRAC_PI_2
            };
            let alt = pos.z.abs() - b;
            return (lat.to_degrees(), lon.to_degrees(), alt);
        }

        // Bowring 迭代求大地纬度
        let mut lat = (pos.z / p).atan(); // 初始：球心纬度
        for _ in 0..6 {
            let sin_lat = lat.sin();
            let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt(); // 卯酉圈曲率半径
            let alt = p / lat.cos() - n; // h 通过当前 lat 求
            lat = (pos.z / p * (1.0 - e2 * n / (n + alt)).recip()).atan();
        }
        // 由最终 lat 精确反算高度
        let sin_lat = lat.sin();
        let cos_lat = lat.cos();
        let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
        let alt = if cos_lat > 1e-9 {
            p / cos_lat - n
        } else {
            pos.z.abs() - b
        };
        (lat.to_degrees(), lon.to_degrees(), alt)
    }

    /// 将地理坐标（大地纬度、经度、椭球高度）转为 ECEF（椭球正算）
    ///
    /// 与 `ecef_to_geo` 严格互逆（同一 WGS-84 椭球参数）。
    pub fn geo_to_ecef(lat_deg: f64, lon_deg: f64, alt: f64) -> Vec3 {
        let a = EARTH_EQUATORIAL_RADIUS;
        let b = EARTH_POLAR_RADIUS;
        let e2 = 1.0 - (b * b) / (a * a);
        let lat = lat_deg.to_radians();
        let lon = lon_deg.to_radians();
        let sin_lat = lat.sin();
        let cos_lat = lat.cos();
        let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt(); // 卯酉圈曲率半径
        Vec3::new(
            (n + alt) * cos_lat * lon.cos(),
            (n + alt) * cos_lat * lon.sin(),
            (n * (1.0 - e2) + alt) * sin_lat,
        )
    }

    pub fn great_circle_distance(
        lat1_deg: f64,
        lon1_deg: f64,
        lat2_deg: f64,
        lon2_deg: f64,
    ) -> f64 {
        let (lat1, lon1) = (lat1_deg.to_radians(), lon1_deg.to_radians());
        let (lat2, lon2) = (lat2_deg.to_radians(), lon2_deg.to_radians());
        let dlat = (lat1 - lat2) * 0.5;
        let dlon = (lon1 - lon2) * 0.5;
        let a = (dlat.sin()).powi(2) + lat1.cos() * lat2.cos() * (dlon.sin()).powi(2);
        let c = 2.0 * a.sqrt().asin();
        c * EARTH_EQUATORIAL_RADIUS
    }

    /// 时间步进
    pub fn step(&mut self, dt: f64) {
        if matches!(self.phase, IcbmPhase::Impact | IcbmPhase::Failed) {
            return;
        }

        // 当前质量
        let total_mass = self.current_mass();
        if total_mass <= 0.0 {
            self.phase = IcbmPhase::Failed;
            return;
        }

        // 1. 重力（球形点质量，暂不含 J2）
        let r = self.position_ecef.length();
        let grav_accel = if r > 0.0 {
            self.position_ecef / r * (-EARTH_MU / (r * r))
        } else {
            Vec3::zero()
        };

        // 2. 柯里奥利 + 离心
        let coriolis = EarthModel::coriolis_accel(self.velocity_ecef);

        // 3. 推力
        let thrust = self.current_thrust();
        let thrust_dir = self.thrust_direction();
        let thrust_accel = if total_mass > 0.0 {
            thrust_dir * (thrust / total_mass)
        } else {
            Vec3::zero()
        };

        // 4. 大气阻力 (仅低高度)
        let drag_accel = self.compute_drag();

        let accel = grav_accel + coriolis + thrust_accel + drag_accel;
        self.velocity_ecef += accel * dt;
        self.position_ecef += self.velocity_ecef * dt;

        self.flight_time += dt;
        self.stage_time += dt;

        self.consume_propellant(dt);
        self.update_phase(dt);

        // 9. 更新弹道参数
        let (lat, lon, alt) = Self::ecef_to_geo(self.position_ecef);
        let target_dist = Self::great_circle_distance(
            lat,
            lon,
            self.config.target_lat_deg,
            self.config.target_lon_deg,
        );
        self.range_to_target_m = target_dist;

        if alt > self.apogee_m {
            self.apogee_m = alt;
        }

        // 检查是否撞击
        if alt <= 0.0 && self.flight_time > 10.0 {
            self.phase = IcbmPhase::Impact;
        }
    }

    fn current_mass(&self) -> f64 {
        let remaining: f64 = self.propellant_remaining.iter().sum();
        let structure: f64 = self
            .config
            .stages
            .iter()
            .map(|s| {
                if matches!(self.phase, IcbmPhase::Boost(i))
                    || matches!(self.phase, IcbmPhase::Coast)
                    || matches!(self.phase, IcbmPhase::Midcourse)
                    || matches!(self.phase, IcbmPhase::BusDeployment)
                {
                    s.structural_mass_kg
                } else {
                    0.0
                }
            })
            .sum();

        // 简化：所有级结构质量保留
        let stage_structure: f64 = self
            .config
            .stages
            .iter()
            .map(|s| s.structural_mass_kg)
            .sum();
        let rv_mass: f64 = self
            .config
            .reentry_vehicles
            .iter()
            .map(|rv| rv.mass_kg)
            .sum();
        remaining + stage_structure + rv_mass + self.config.bus_mass_kg
    }

    /// 当前推力 (N)
    fn current_thrust(&self) -> f64 {
        match self.phase {
            IcbmPhase::Boost(i) => {
                if i >= self.config.stages.len() {
                    return 0.0;
                }
                let stage = &self.config.stages[i];
                // 高度修正推力
                let alt = self.position_ecef.length() - EARTH_EQUATORIAL_RADIUS;
                let p_amb = if alt < 0.0 {
                    101325.0
                } else if alt < 100_000.0 {
                    // 简化大气压
                    101325.0 * (-alt / 8500.0).exp()
                } else {
                    0.0
                };
                let thrust = stage.thrust_vac_n - p_amb * stage.exit_area_m2;
                thrust.max(0.0)
            }
            _ => 0.0,
        }
    }

    /// 到目标的方位角（从当前位置看向目标，正北顺时针）
    fn bearing_to_target(&self) -> f64 {
        let (lat1_deg, lon1_deg, _) = Self::ecef_to_geo(self.position_ecef);
        let lat1 = lat1_deg.to_radians();
        let lon1 = lon1_deg.to_radians();
        let lat2 = self.config.target_lat_deg.to_radians();
        let lon2 = self.config.target_lon_deg.to_radians();
        let dlon = lon2 - lon1;
        let y = dlon.sin() * lat2.cos();
        let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos();
        y.atan2(x).to_degrees()
    }

    fn thrust_direction(&self) -> Vec3 {
        let vel = self.velocity_ecef;
        let speed = vel.length();

        // 本地"向上"方向（从地心指向当前位置）
        let up = self.position_ecef.normalized();

        // 本地东向、北向（ENU 坐标系）
        let east = {
            let e = Vec3::new(0.0, 0.0, 1.0).cross(&up);
            if e.length_squared() < 1e-10 {
                Vec3::new(1.0, 0.0, 0.0)
            } else {
                e.normalized()
            }
        };
        let north = east.cross(&up);

        if matches!(self.phase, IcbmPhase::Boost(_)) {
            // === ICBM 重力转弯制导 ===
            // 阶段 1 (0-10s)：垂直上升，建立速度
            // 阶段 2 (10-30s)：向目标方位逐步倾斜
            // 阶段 3 (30s+)：保持 25° 仰角瞄准目标

            let pitch_angle_deg = if self.flight_time < 12.0 {
                // 垂直段
                90.0
            } else if self.flight_time < 72.0 {
                // 缓慢倾倒：powf(1.5) 曲线，优先爬高
                let frac = ((self.flight_time - 12.0) / 60.0).min(1.0);
                90.0 - 70.0 * frac.powf(1.5)
            } else {
                20.0
            };

            // 俯仰比例：0°=纯水平, 90°=纯垂直
            let pitch_frac = 1.0 - (pitch_angle_deg / 90.0).clamp(0.0, 1.0);

            // 目标方位（持续更新）
            let current_bearing = self.bearing_to_target();
            let hdg_rad = current_bearing.to_radians();
            let horiz_dir = north * hdg_rad.cos() + east * hdg_rad.sin();

            // 合成：up 与 horiz_dir 按 pitch_frac 混合
            let dir = up * (1.0 - pitch_frac) + horiz_dir * pitch_frac;
            if dir.length_squared() > 1e-10 {
                dir.normalized()
            } else {
                up
            }
        } else if speed > 1.0 {
            // 非助推段沿速度方向
            vel.normalized()
        } else {
            up
        }
    }

    fn compute_drag(&self) -> Vec3 {
        let speed = self.velocity_ecef.length();
        if speed < 50.0 {
            return Vec3::zero();
        }
        let alt = self.position_ecef.length() - EARTH_EQUATORIAL_RADIUS;
        if alt > 120_000.0 {
            return Vec3::zero();
        }
        // 简化的密度
        let rho = if alt < 0.0 {
            1.225
        } else {
            1.225 * (-alt / 8500.0).exp()
        };
        // 风场：ENU → ECEF
        use crate::aerodynamics::WindField;
        let wind_enu = WindField::default().wind_at(alt);
        let (lat, lon, _) = Self::ecef_to_geo(self.position_ecef);
        let (slat, clat) = lat.sin_cos();
        let (slon, clon) = lon.sin_cos();
        // ENU 基向量在 ECEF 中
        let east = Vec3::new(-slon, clon, 0.0);
        let north = Vec3::new(-slat * clon, -slat * slon, clat);
        let wind_ecef = east * wind_enu.x + north * wind_enu.y;
        // 空速
        let air_vel = self.velocity_ecef - wind_ecef;
        let air_speed = air_vel.length().max(1.0);
        let cd = 0.2;
        let area = 1.0;
        let drag_mag = 0.5 * rho * air_speed * air_speed * cd * area;
        let total_mass = self.current_mass();
        if total_mass <= 0.0 {
            return Vec3::zero();
        }
        -air_vel.normalized() * (drag_mag / total_mass)
    }

    fn consume_propellant(&mut self, dt: f64) {
        match self.phase {
            IcbmPhase::Boost(i) => {
                if i < self.propellant_remaining.len() {
                    let stage = &self.config.stages[i];
                    let isp = if self.position_ecef.length() - EARTH_EQUATORIAL_RADIUS > 30_000.0 {
                        stage.isp_vac_s
                    } else {
                        stage.isp_sl_s.max(260.0)
                    };
                    let mass_flow = stage.thrust_vac_n / (isp * 9.80665);
                    self.propellant_remaining[i] =
                        (self.propellant_remaining[i] - mass_flow * dt).max(0.0);
                }
            }
            _ => {}
        }
    }

    fn update_phase(&mut self, _dt: f64) {
        match self.phase {
            IcbmPhase::Boost(i) => {
                if self.propellant_remaining[i] <= 0.0
                    || self.stage_time >= self.config.stages[i].burn_duration_s
                {
                    // 级间分离
                    let next_stage = i + 1;
                    if next_stage < self.config.stages.len() {
                        self.current_stage = next_stage;
                        self.stage_time = 0.0;
                        self.phase = IcbmPhase::Boost(next_stage);
                    } else {
                        // 最后一节熄火 → 进入中段
                        if self.flight_time > 30.0 {
                            self.phase = IcbmPhase::Midcourse;
                        } else {
                            self.phase = IcbmPhase::Coast;
                        }
                    }
                }
            }
            IcbmPhase::Coast => {
                self.phase = IcbmPhase::Midcourse;
            }
            IcbmPhase::Midcourse => {
                // 在远地点附近部署 MIRV
                let (_, _, alt) = Self::ecef_to_geo(self.position_ecef);
                if alt > 500_000.0 && alt < 1_500_000.0 {
                    self.phase = IcbmPhase::BusDeployment;
                }
            }
            IcbmPhase::BusDeployment => {
                // 部署弹头：按时间间隔释放
                for (i, deployed) in self.rv_deployed.iter_mut().enumerate() {
                    if !*deployed {
                        if i as f64 * 0.5 < self.stage_time {
                            *deployed = true;
                        }
                    }
                }
                // 所有弹头部署完毕 → 再入
                if self.rv_deployed.iter().all(|&d| d) {
                    self.phase = IcbmPhase::Reentry;
                }
            }
            IcbmPhase::Reentry => {
                // 再入由阻力自动处理，撞击检测在 step 中
            }
            _ => {}
        }
    }
}

// =====================================================================
// CEP 计算
// =====================================================================

/// 根据关机点速度/位置误差估算 CEP
pub fn estimate_cep(
    downrange_km: f64,
    velocity_error_mps: f64,
    position_error_m: f64,
    guidance_factor: f64,
) -> f64 {
    // 简化模型：射程越远、误差越大 → CEP 越大
    let base = (velocity_error_mps * 10.0 + position_error_m * 0.5) * (downrange_km / 5000.0);
    base * guidance_factor
}

// =====================================================================
// 测试
// =====================================================================
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn earth_radius() {
        let r_eq = EarthModel::radius_at_latitude(0.0);
        let r_pole = EarthModel::radius_at_latitude(90.0);
        assert!((r_eq - 6_378_137.0).abs() < 1000.0);
        assert!(r_eq > r_pole);
    }

    #[test]
    fn icbm_config() {
        let mm3 = IcbmConfig::minuteman3();
        assert!(mm3.total_propellant_mass() > 30_000.0);
        assert_eq!(mm3.stages.len(), 3);
        assert_eq!(mm3.reentry_vehicles.len(), 3);
    }

    #[test]
    fn ecef_geo_roundtrip() {
        // 给定大地坐标（纬度/经度/椭球高度），经 geo_to_ecef → ecef_to_geo 应恒等
        let (lat, lon, alt): (f64, f64, f64) = (40.0, 116.0, 500.0);
        let pos = IcbmState::geo_to_ecef(lat, lon, alt);
        let (lat2, lon2, alt2) = IcbmState::ecef_to_geo(pos);
        assert!((lat - lat2).abs() < 0.001, "lat: {} vs {}", lat, lat2);
        assert!((lon - lon2).abs() < 0.001, "lon: {} vs {}", lon, lon2);
        assert!((alt - alt2).abs() < 1.0, "alt: {} vs {}", alt, alt2);

        // 再验证一个南半球点
        let (lat3, lon3, alt3) = (-35.0, -70.0, 10_000.0);
        let pos3 = IcbmState::geo_to_ecef(lat3, lon3, alt3);
        let (lat4, lon4, alt4) = IcbmState::ecef_to_geo(pos3);
        assert!((lat3 - lat4).abs() < 0.001);
        assert!((lon3 - lon4).abs() < 0.001);
        assert!((alt3 - alt4).abs() < 1.0);
    }

    #[test]
    fn great_circle_distance_known() {
        // 北京到上海约 1000km
        let d = IcbmState::great_circle_distance(39.9, 116.4, 31.2, 121.5);
        assert!(d > 900_000.0 && d < 1_200_000.0);
    }

    #[test]
    fn icbm_launch() {
        let cfg = IcbmConfig::minuteman3();
        let mut icbm = IcbmState::new(cfg);
        icbm.launch();
        assert!(matches!(icbm.phase, IcbmPhase::Boost(0)));
        // 模拟几秒
        for _ in 0..500 {
            icbm.step(0.1);
            if matches!(icbm.phase, IcbmPhase::Impact | IcbmPhase::Failed) {
                break;
            }
        }
        assert!(icbm.flight_time > 5.0);
    }

    #[test]
    fn cep_estimation() {
        let cep = estimate_cep(5000.0, 5.0, 50.0, 1.0);
        assert!(cep > 0.0);
    }

    #[test]
    fn coriolis() {
        let vel = Vec3::new(7000.0, 0.0, 0.0);
        let a = EarthModel::coriolis_accel(vel);
        assert!(a.length() > 0.0);
    }
}
