//! 环境模型：行星大气 (US Standard 1976)、热模拟、损伤系统
use crate::{Vec3, G};

// =====================================================================
// 大气模型 (US Standard Atmosphere 1976)
// =====================================================================
pub struct Atmosphere {
    #[allow(dead_code)]
    sea_level_pressure: f64,
    #[allow(dead_code)]
    scale_height: f64,
}

struct LayerState {
    pressure_pa: f64,
    temperature_k: f64,
}

impl Atmosphere {
    pub fn new(sea_level_pressure: f64, scale_height: f64) -> Self {
        Self {
            sea_level_pressure,
            scale_height,
        }
    }

    pub fn get_pressure(&self, altitude: f64) -> f64 {
        let h = altitude.max(0.0);
        self.evaluate_isa(h).pressure_pa.max(0.0)
    }

    pub fn get_density(&self, altitude: f64) -> f64 {
        let h = altitude.max(0.0);
        let s = self.evaluate_isa(h);
        if s.temperature_k <= 0.0 {
            0.0
        } else {
            (s.pressure_pa / (crate::AIR_GAS_CONSTANT * s.temperature_k)).max(0.0)
        }
    }

    pub fn get_temperature(&self, altitude: f64) -> f64 {
        self.evaluate_isa(altitude.max(0.0)).temperature_k
    }

    pub fn get_speed_of_sound(&self, altitude: f64) -> f64 {
        let tk = self.get_temperature(altitude);
        if tk <= 0.0 {
            0.0
        } else {
            (crate::GAMMA_AIR * crate::AIR_GAS_CONSTANT * tk).sqrt()
        }
    }

    fn evaluate_isa(&self, altitude: f64) -> LayerState {
        const G0: f64 = 9.80665;
        const R: f64 = 8.3144598;
        const M: f64 = 0.0289644;
        const EXP_SCALE: f64 = (G0 * M) / R;

        // [hBase, hTop, tBase, pBase, lapse]
        const LAYERS: [(f64, f64, f64, f64, f64); 7] = [
            (0.0, 11000.0, 288.15, 101325.0, -0.0065),
            (11000.0, 20000.0, 216.65, 22632.06, 0.0),
            (20000.0, 32000.0, 216.65, 5474.889, 0.001),
            (32000.0, 47000.0, 228.65, 868.0187, 0.0028),
            (47000.0, 51000.0, 270.65, 110.9063, 0.0),
            (51000.0, 71000.0, 270.65, 66.93887, -0.0028),
            (71000.0, 84852.0, 214.65, 3.956420, -0.002),
        ];

        for &(hb, ht, tb, pb, lapse) in LAYERS.iter() {
            if altitude <= ht {
                let dh = altitude - hb;
                if lapse.abs() < 1e-12 {
                    return LayerState {
                        pressure_pa: pb * (-(EXP_SCALE * dh) / tb).exp(),
                        temperature_k: tb,
                    };
                }
                let temp = tb + lapse * dh;
                let press = pb * (tb / temp).powf(EXP_SCALE / lapse);
                return LayerState {
                    pressure_pa: press,
                    temperature_k: temp,
                };
            }
        }
        // 外逸层延伸
        const H_TOP: f64 = 84852.0;
        const T_TOP: f64 = 186.946;
        const P_TOP: f64 = 0.3734;
        let dh = altitude - H_TOP;
        LayerState {
            pressure_pa: P_TOP * (-(EXP_SCALE * dh) / T_TOP).exp(),
            temperature_k: T_TOP,
        }
    }
}

// =====================================================================
// 行星
// =====================================================================
pub struct Planet {
    name: String,
    mass: f64,
    radius: f64,
    atmosphere: Atmosphere,
}

impl Planet {
    pub fn new(name: &str, mass: f64, radius: f64, atmosphere: Atmosphere) -> Self {
        Self {
            name: name.to_string(),
            mass,
            radius,
            atmosphere,
        }
    }

    pub fn get_gravity_at(&self, position: Vec3) -> Vec3 {
        let r = position.length();
        if r == 0.0 {
            return Vec3::zero();
        }
        let g_mag = G * self.mass / (r * r);
        position.normalized() * (-g_mag)
    }

    pub fn get_altitude(&self, position: Vec3) -> f64 {
        position.length() - self.radius
    }

    pub fn get_atmosphere(&self) -> &Atmosphere {
        &self.atmosphere
    }
    pub fn get_radius(&self) -> f64 {
        self.radius
    }
    pub fn get_mass(&self) -> f64 {
        self.mass
    }
    pub fn get_name(&self) -> &str {
        &self.name
    }
}

// =====================================================================
// 损伤系统
// =====================================================================
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DamageType {
    Tps,
    Structural,
    Propulsion,
    LifeSupport,
}

#[derive(Debug, Clone)]
pub struct DamageComponent {
    pub damage_tps: f64,
    pub damage_structural: f64,
    pub damage_propulsion: f64,
    pub damage_life_support: f64,
    pub cabin_temperature: f64,
    pub cabin_pressure: f64,
    pub oxygen_level: f64,
    pub co2_level: f64,
}

impl DamageComponent {
    pub fn new() -> Self {
        Self {
            damage_tps: 0.0,
            damage_structural: 0.0,
            damage_propulsion: 0.0,
            damage_life_support: 0.0,
            cabin_temperature: 293.15,
            cabin_pressure: 101.325,
            oxygen_level: 0.209,
            co2_level: 0.0,
        }
    }

    pub fn total_damage(&self) -> f64 {
        (self.damage_tps
            + self.damage_structural
            + self.damage_propulsion
            + self.damage_life_support)
            .min(1.0)
    }

    pub fn vessel_health(&self) -> f64 {
        1.0 - self.total_damage()
    }

    pub fn apply_damage(&mut self, dmg_type: DamageType, amount: f64) {
        let target = match dmg_type {
            DamageType::Tps => &mut self.damage_tps,
            DamageType::Structural => &mut self.damage_structural,
            DamageType::Propulsion => &mut self.damage_propulsion,
            DamageType::LifeSupport => &mut self.damage_life_support,
        };
        *target = (*target + amount).min(1.0);
    }
}

pub struct DamageSystem;

impl DamageSystem {
    pub fn update(dt: f64, damage: &mut DamageComponent, damage_factor: f64) {
        // 自愈：缓慢恢复（模拟维修效果）
        if damage.damage_tps > 0.0 && damage_factor < 0.5 {
            damage.damage_tps = (damage.damage_tps - 0.001 * dt).max(0.0);
        }
        if damage.damage_structural > 0.0 {
            damage.damage_structural = (damage.damage_structural - 0.0005 * dt).max(0.0);
        }
    }

    pub fn survival_probability(damage: &DamageComponent) -> f64 {
        let d = damage.total_damage();
        if d > 0.8 {
            return 0.0;
        }
        if d > 0.6 {
            return (0.8 - d) / 0.2;
        }
        1.0 - d * 0.5
    }
}

// =====================================================================
// 热模拟（含烧蚀损伤）
// =====================================================================
pub struct ThermalSimulation {
    heat_flux: f64,
    total_heat: f64,
    peak_heat_flux: f64,
    tps_ablation: f64, // 累积 TPS 烧蚀量 [0..1]
    stagnation_pressure: f64,
    config: crate::simulation::ThermalConfig,
}

impl ThermalSimulation {
    pub fn new(config: crate::simulation::ThermalConfig) -> Self {
        Self {
            heat_flux: 0.0,
            total_heat: 0.0,
            peak_heat_flux: 0.0,
            tps_ablation: 0.0,
            stagnation_pressure: 0.0,
            config,
        }
    }

    /// 更新热通量，使用 Sutton-Graves 驻点加热公式
    pub fn update(&mut self, dt: f64, speed: f64, density: f64, integrity: f64) {
        // Sutton-Graves q = k * sqrt(rho/R_nose) * v^3
        let q_sg = self.config.sutton_graves_k * (density / self.config.nose_radius_m).sqrt() * speed.powi(3);

        // 结合 integrity（TPS 损伤越严重，热流倍数效应越强）
        let c_h = self.config.convection_coefficient * (1.0 + self.config.damage_heat_multiplier * (1.0 - integrity).max(0.0));
        let q_simple = 0.5 * density * speed.powi(3) * c_h;

        self.heat_flux = q_sg.max(q_simple).max(0.0);
        self.total_heat += self.heat_flux * dt;

        // 驻点压力
        self.stagnation_pressure = 0.5 * density * speed * speed;

        if self.heat_flux > self.peak_heat_flux {
            self.peak_heat_flux = self.heat_flux;
        }
    }

    /// TPS 烧蚀模型：当热流超过阈值时，TPS 逐渐烧蚀
    pub fn ablate(&mut self, dt: f64, threshold: f64) -> f64 {
        let excess = self.heat_flux - threshold;
        if excess > 0.0 {
            // 烧蚀率：超出阈值越多，烧蚀越快
            // 约在 500 kW/m² 时 0.033/s（默认 ablation_rate_coefficient=6.67e-8）
            let rate = excess * self.config.ablation_rate_coefficient;
            self.tps_ablation = (self.tps_ablation + rate * dt).min(1.0);
            rate * dt
        } else {
            0.0
        }
    }

    pub fn get_heat_flux(&self) -> f64 {
        self.heat_flux
    }
    pub fn get_total_heat(&self) -> f64 {
        self.total_heat
    }
    pub fn get_peak_heat_flux(&self) -> f64 {
        self.peak_heat_flux
    }
    pub fn get_tps_ablation(&self) -> f64 {
        self.tps_ablation
    }
    pub fn get_stagnation_pressure(&self) -> f64 {
        self.stagnation_pressure
    }
    pub fn reset_ablation(&mut self) {
        self.tps_ablation = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn atmosphere_sea_level() {
        let atmo = Atmosphere::new(101325.0, 8500.0);
        let p = atmo.get_pressure(0.0);
        assert!((p - 101325.0).abs() < 100.0);
        let t = atmo.get_temperature(0.0);
        assert!((t - 288.15).abs() < 0.5);
        let d = atmo.get_density(0.0);
        assert!(d > 1.0 && d < 1.3);
    }

    #[test]
    fn atmosphere_tropopause() {
        let atmo = Atmosphere::new(101325.0, 8500.0);
        let t = atmo.get_temperature(11000.0);
        assert!((t - 216.65).abs() < 0.5);
        let p = atmo.get_pressure(11000.0);
        assert!((p - 22632.06).abs() / 22632.06 < 0.01);
    }

    #[test]
    fn atmosphere_stratosphere() {
        let atmo = Atmosphere::new(101325.0, 8500.0);
        let t = atmo.get_temperature(30000.0);
        assert!((t - 226.65).abs() < 1.0);
    }

    #[test]
    fn atmosphere_high_altitude() {
        let atmo = Atmosphere::new(101325.0, 8500.0);
        let t = atmo.get_temperature(100_000.0);
        assert!(t > 180.0 && t < 200.0);
    }

    #[test]
    fn atmosphere_speed_of_sound_sl() {
        let atmo = Atmosphere::new(101325.0, 8500.0);
        let sos = atmo.get_speed_of_sound(0.0);
        assert!((sos - 340.0).abs() < 5.0);
    }

    #[test]
    fn planet_gravity() {
        let atmo = Atmosphere::new(101325.0, 8500.0);
        let earth = Planet::new("Earth", 5.9722e24, 6_371_000.0, atmo);
        let g = earth.get_gravity_at(Vec3::new(0.0, 6_371_000.0, 0.0));
        // g ≈ 9.81 m/s² at surface
        assert!((g.length() - 9.81).abs() < 0.02);
        assert!(g.y < 0.0); // 指向地心
    }

    #[test]
    fn planet_altitude() {
        let atmo = Atmosphere::new(101325.0, 8500.0);
        let earth = Planet::new("Earth", 5.9722e24, 6_371_000.0, atmo);
        let alt = earth.get_altitude(Vec3::new(0.0, 6_571_000.0, 0.0));
        assert!((alt - 200_000.0).abs() < 0.001);
    }

    #[test]
    fn planet_getters() {
        let atmo = Atmosphere::new(101325.0, 8500.0);
        let earth = Planet::new("Earth", 5.9722e24, 6_371_000.0, atmo);
        assert_eq!(earth.get_name(), "Earth");
        assert!((earth.get_mass() - 5.9722e24).abs() < 1.0e19);
        assert!((earth.get_radius() - 6_371_000.0).abs() < 1.0);
    }

    #[test]
    fn damage_accumulation() {
        let mut dmg = DamageComponent::new();
        assert!((dmg.vessel_health() - 1.0).abs() < 1e-12);
        dmg.apply_damage(DamageType::Tps, 0.3);
        assert!((dmg.damage_tps - 0.3).abs() < 1e-12);
        dmg.apply_damage(DamageType::Structural, 0.2);
        assert!((dmg.total_damage() - 0.5).abs() < 1e-12);
    }

    #[test]
    fn damage_clamped() {
        let mut dmg = DamageComponent::new();
        dmg.apply_damage(DamageType::Tps, 1.5);
        assert!((dmg.damage_tps - 1.0).abs() < 1e-12);
    }

    #[test]
    fn damage_survival() {
        let mut dmg = DamageComponent::new();
        assert!((DamageSystem::survival_probability(&dmg) - 1.0).abs() < 1e-12);
        dmg.apply_damage(DamageType::Tps, 0.4);
        dmg.apply_damage(DamageType::Structural, 0.3);
        let sp = DamageSystem::survival_probability(&dmg);
        assert!(sp > 0.0 && sp < 1.0);
    }

    #[test]
    fn thermal_simulation() {
        let mut t = ThermalSimulation::new(crate::simulation::ThermalConfig::default());
        assert!((t.get_heat_flux() - 0.0).abs() < 1e-12);
        t.update(1.0, 7800.0, 1.2, 1.0);
        // 高超声速再入应有显著热流
        assert!(t.get_heat_flux() > 0.0);
        assert!(t.get_total_heat() > 0.0);
        assert!(t.get_peak_heat_flux() > 0.0);
    }

    #[test]
    fn thermal_peak_tracking() {
        let mut t = ThermalSimulation::new(crate::simulation::ThermalConfig::default());
        t.update(1.0, 100.0, 1.0, 1.0);
        let peak1 = t.get_peak_heat_flux();
        t.update(1.0, 1000.0, 1.0, 1.0);
        assert!(t.get_peak_heat_flux() > peak1);
    }
}
