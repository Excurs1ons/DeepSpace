//! 传感器与电子战模块
//!
//! 提供雷达、IRST、RWR、ECM 等传感器建模。
//! 用于超视距（BVR）空战中的探测、跟踪、对抗。

use crate::Vec3;

// =====================================================================
// 雷达模型
// =====================================================================

/// 雷达工作模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RadarMode {
    Rws,
    Tws,
    Stt,
    AirToGround,
    Off,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RadarType {
    PulseDoppler,
    Aesa,
    Pesa,
}

#[derive(Debug, Clone)]
pub struct RadarConfig {
    pub name: String,
    pub radar_type: RadarType,
    pub peak_power_w: f64,
    pub antenna_gain_db: f64,
    pub frequency_hz: f64,
    pub bandwidth_hz: f64,
    pub noise_figure_db: f64,
    pub pulse_width_us: f64,
    pub prf_hz: f64,
    pub beam_width_deg: f64,
    pub scan_speed_deg_s: f64,
    pub max_gimbal_deg: f64,
    pub min_snr_db: f64,
    pub range_km: f64,
    pub track_capacity: i32,
    pub update_rate_hz: f64,
}

impl RadarConfig {
    pub fn apg77() -> Self {
        RadarConfig {
            name: "AN/APG-77".into(),
            radar_type: RadarType::Aesa,
            peak_power_w: 20_000.0,
            antenna_gain_db: 35.0,
            frequency_hz: 10e9,
            bandwidth_hz: 5e6,
            noise_figure_db: 3.5,
            pulse_width_us: 1.0,
            prf_hz: 100_000.0,
            beam_width_deg: 2.0,
            scan_speed_deg_s: 100.0,
            max_gimbal_deg: 60.0,
            min_snr_db: 13.0,
            range_km: 240.0,
            track_capacity: 30,
            update_rate_hz: 10.0,
        }
    }
    pub fn apg80() -> Self {
        RadarConfig {
            name: "AN/APG-80".into(),
            radar_type: RadarType::Aesa,
            peak_power_w: 15_000.0,
            antenna_gain_db: 33.0,
            frequency_hz: 10e9,
            bandwidth_hz: 5e6,
            noise_figure_db: 4.0,
            pulse_width_us: 1.0,
            prf_hz: 80_000.0,
            beam_width_deg: 2.5,
            scan_speed_deg_s: 100.0,
            max_gimbal_deg: 60.0,
            min_snr_db: 14.0,
            range_km: 200.0,
            track_capacity: 20,
            update_rate_hz: 10.0,
        }
    }
    pub fn irbis_e() -> Self {
        RadarConfig {
            name: "IRBIS-E".into(),
            radar_type: RadarType::Pesa,
            peak_power_w: 20_000.0,
            antenna_gain_db: 32.0,
            frequency_hz: 10e9,
            bandwidth_hz: 6e6,
            noise_figure_db: 4.5,
            pulse_width_us: 1.5,
            prf_hz: 60_000.0,
            beam_width_deg: 2.0,
            scan_speed_deg_s: 80.0,
            max_gimbal_deg: 60.0,
            min_snr_db: 14.0,
            range_km: 200.0,
            track_capacity: 30,
            update_rate_hz: 8.0,
        }
    }
    pub fn type1475() -> Self {
        RadarConfig {
            name: "Type 1475".into(),
            radar_type: RadarType::Aesa,
            peak_power_w: 24_000.0,
            antenna_gain_db: 36.0,
            frequency_hz: 10e9,
            bandwidth_hz: 5e6,
            noise_figure_db: 3.0,
            pulse_width_us: 0.5,
            prf_hz: 120_000.0,
            beam_width_deg: 1.8,
            scan_speed_deg_s: 120.0,
            max_gimbal_deg: 65.0,
            min_snr_db: 12.0,
            range_km: 260.0,
            track_capacity: 40,
            update_rate_hz: 12.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Radar {
    pub config: RadarConfig,
    pub mode: RadarMode,
    pub is_active: bool,
    pub azimuth_deg: f64,
    pub elevation_deg: f64,
}

impl Radar {
    pub fn new(config: RadarConfig) -> Self {
        Radar {
            config,
            mode: RadarMode::Off,
            is_active: false,
            azimuth_deg: 0.0,
            elevation_deg: 0.0,
        }
    }

    pub fn detection_range_m(
        &self,
        target_rcs_m2: f64,
        relative_speed: f64,
        altitude_diff: f64,
    ) -> f64 {
        if !self.is_active || self.mode == RadarMode::Off {
            return 0.0;
        }
        let rcs_factor = (target_rcs_m2 / 5.0).sqrt().sqrt();
        let base = self.config.range_km * 1000.0 * rcs_factor;
        let dop_factor = if relative_speed.abs() < 50.0 {
            0.5
        } else if relative_speed.abs() < 100.0 {
            0.7
        } else {
            1.0
        };
        let ld_factor = if altitude_diff < -1000.0 {
            0.7
        } else if altitude_diff > 1000.0 {
            1.0
        } else {
            0.85
        };
        let off_deg = self.azimuth_deg.abs().max(self.elevation_deg.abs());
        let beam_factor = if off_deg < self.config.beam_width_deg {
            1.0
        } else if off_deg < 30.0 {
            0.8 - off_deg * 0.01
        } else if off_deg < 60.0 {
            0.3
        } else {
            0.0
        };
        base * dop_factor * ld_factor * beam_factor
    }

    pub fn detection_probability(&self, range_m: f64, target_rcs_m2: f64) -> f64 {
        let max_r = self.detection_range_m(target_rcs_m2, 200.0, 3000.0);
        if max_r <= 0.0 || range_m >= max_r * 1.5 {
            return 0.0;
        }
        if range_m <= max_r * 0.5 {
            return 1.0;
        }
        let t = (range_m / max_r - 0.5) * 4.0;
        1.0 / (1.0 + (-t).exp())
    }

    pub fn set_mode(&mut self, mode: RadarMode) {
        self.mode = mode;
        self.is_active = mode != RadarMode::Off;
    }

    pub fn scan(&mut self, az: f64, el: f64) {
        self.azimuth_deg = az;
        self.elevation_deg = el;
    }
}

// =====================================================================
// IRST
// =====================================================================

#[derive(Debug, Clone)]
pub struct IrstConfig {
    pub name: String,
    pub detection_range_km: f64,
    pub fov_deg: f64,
    pub slew_rate_deg_s: f64,
    pub wavelength_um: f64,
}

impl IrstConfig {
    pub fn eots() -> Self {
        IrstConfig {
            name: "EOTS".into(),
            detection_range_km: 100.0,
            fov_deg: 30.0,
            slew_rate_deg_s: 60.0,
            wavelength_um: 2.5,
        }
    }
    pub fn ols35() -> Self {
        IrstConfig {
            name: "OLS-35".into(),
            detection_range_km: 90.0,
            fov_deg: 30.0,
            slew_rate_deg_s: 60.0,
            wavelength_um: 3.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Irst {
    pub config: IrstConfig,
    pub is_active: bool,
}

impl Irst {
    pub fn new(config: IrstConfig) -> Self {
        Irst {
            config,
            is_active: false,
        }
    }

    pub fn detection_range_m(&self, afterburner: bool, target_speed: f64) -> f64 {
        if !self.is_active {
            return 0.0;
        }
        let base = self.config.detection_range_km * 1000.0;
        let eng_factor = if afterburner { 1.8 } else { 1.0 };
        let spd_factor = 1.0 + (target_speed / 340.0 - 1.0).max(0.0) * 0.2;
        base * eng_factor * spd_factor.min(2.0)
    }
}

// =====================================================================
// RWR
// =====================================================================

#[derive(Debug, Clone)]
pub struct RwrConfig {
    pub name: String,
    pub freq_min_hz: f64,
    pub freq_max_hz: f64,
    pub bearing_accuracy_deg: f64,
    pub sensitivity_db_w_m2: f64,
}

impl RwrConfig {
    pub fn alr94() -> Self {
        RwrConfig {
            name: "AN/ALR-94".into(),
            freq_min_hz: 0.5e9,
            freq_max_hz: 20e9,
            bearing_accuracy_deg: 1.0,
            sensitivity_db_w_m2: -70.0,
        }
    }
    pub fn l150() -> Self {
        RwrConfig {
            name: "L-150 Pastel".into(),
            freq_min_hz: 1e9,
            freq_max_hz: 18e9,
            bearing_accuracy_deg: 3.0,
            sensitivity_db_w_m2: -65.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct RwrContact {
    pub bearing_deg: f64,
    pub frequency_hz: f64,
    pub estimated_radar_type: String,
    pub signal_strength: f64,
}

#[derive(Debug, Clone)]
pub struct Rwr {
    pub config: RwrConfig,
    pub is_active: bool,
    pub contacts: Vec<RwrContact>,
}

impl Rwr {
    pub fn new(config: RwrConfig) -> Self {
        Rwr {
            config,
            is_active: false,
            contacts: Vec::new(),
        }
    }
    pub fn detect(&mut self, freq: f64, power_w: f64, range_m: f64) -> Option<RwrContact> {
        if !self.is_active || freq < self.config.freq_min_hz || freq > self.config.freq_max_hz {
            return None;
        }
        let p = power_w / (4.0 * std::f64::consts::PI * range_m * range_m);
        if p < 1e-12 {
            return None;
        }
        Some(RwrContact {
            bearing_deg: 0.0,
            frequency_hz: freq,
            estimated_radar_type: String::new(),
            signal_strength: (p * 1e12).min(1.0),
        })
    }
}

// =====================================================================
// ECM
// =====================================================================

#[derive(Debug, Clone)]
pub struct EcmConfig {
    pub name: String,
    pub jammer_power_w: f64,
    pub bandwidth_hz: f64,
    pub max_jam_targets: i32,
}

impl EcmConfig {
    pub fn alq184() -> Self {
        EcmConfig {
            name: "AN/ALQ-184".into(),
            jammer_power_w: 1000.0,
            bandwidth_hz: 4e9,
            max_jam_targets: 4,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Ecm {
    pub config: EcmConfig,
    pub is_active: bool,
    pub enabled: bool,
}

impl Ecm {
    pub fn new(config: EcmConfig) -> Self {
        Ecm {
            config,
            is_active: false,
            enabled: false,
        }
    }
    pub fn jamming_factor(&self, _radar_freq_hz: f64, range_m: f64) -> f64 {
        if !self.enabled || !self.is_active {
            return 1.0;
        }
        0.3 + (range_m / 100_000.0).min(1.0) * 0.7
    }
}

// =====================================================================
// 目标
// =====================================================================

#[derive(Debug, Clone)]
pub struct AirTarget {
    pub id: u32,
    pub position: Vec3,
    pub velocity: Vec3,
    pub rcs_m2: f64,
    pub ir_signature: f64,
    pub is_friendly: bool,
    pub is_afterburner: bool,
    pub is_jamming: bool,
    pub label: String,
}

impl AirTarget {
    pub fn new(id: u32, pos: Vec3, vel: Vec3, rcs: f64, label: &str) -> Self {
        AirTarget {
            id,
            position: pos,
            velocity: vel,
            rcs_m2: rcs,
            ir_signature: 0.5,
            is_friendly: false,
            is_afterburner: false,
            is_jamming: false,
            label: label.into(),
        }
    }
    pub fn range_to(&self, other: &Vec3) -> f64 {
        (self.position - *other).length()
    }
    pub fn closing_speed(&self, observer_vel: Vec3) -> f64 {
        let rel = self.velocity - observer_vel;
        let los = (self.position - observer_vel).normalized();
        -rel.dot(&los)
    }
    pub fn bearing_to(&self, observer_pos: Vec3, observer_heading_deg: f64) -> (f64, f64) {
        let rel = self.position - observer_pos;
        let hr = observer_heading_deg.to_radians();
        let north = rel.x * hr.cos() + rel.y * hr.sin();
        let east = -rel.x * hr.sin() + rel.y * hr.cos();
        let az = east.atan2(north).to_degrees();
        let el = (rel.z / rel.length()).asin().to_degrees();
        (az, el)
    }
}

// =====================================================================
// 测试
// =====================================================================
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn radar_range() {
        let mut r = Radar::new(RadarConfig::apg77());
        r.set_mode(RadarMode::Rws); // 激活雷达（默认 Off）
        assert!(r.detection_range_m(5.0, 200.0, 3000.0) > 200_000.0);
    }
    #[test]
    fn radar_prob() {
        let mut r = Radar::new(RadarConfig::apg77());
        r.set_mode(RadarMode::Rws);
        assert!(r.detection_probability(50_000.0, 5.0) > 0.99);
        assert!(r.detection_probability(400_000.0, 5.0) < 0.1);
    }
    #[test]
    fn radar_stealth() {
        let mut r = Radar::new(RadarConfig::apg77());
        r.set_mode(RadarMode::Rws);
        assert!(
            r.detection_range_m(0.0001, 200.0, 3000.0) < r.detection_range_m(5.0, 200.0, 3000.0)
        );
    }
    #[test]
    fn irst() {
        let mut ir = Irst::new(IrstConfig::eots());
        ir.is_active = true; // 激活红外探测
        assert!(ir.detection_range_m(true, 500.0) > 50_000.0);
    }
    #[test]
    fn rwr() {
        let mut r = Rwr::new(RwrConfig::alr94());
        r.is_active = true;
        assert!(r.detect(10e9, 20_000.0, 100_000.0).is_some());
    }
    #[test]
    fn bearing() {
        let t = AirTarget::new(
            1,
            Vec3::new(100_000.0, 0.0, 10_000.0),
            Vec3::zero(),
            5.0,
            "bogey",
        );
        let (az, _) = t.bearing_to(Vec3::zero(), 0.0);
        assert!(az.abs() < 0.01);
    }
}
