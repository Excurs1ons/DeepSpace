//! 场景系统 — 宇宙沙盘风格 N 体仿真
//!
//! 核心概念：`SceneRuntime` 是可持续运行的仿真器，支持运行时增删天体和
//! 切换场景配置，模拟时钟永不停止。
//!
//! ## 场景文件格式
//! ```ini
//! [scene]
//! name = My Scene
//! dt = 1000.0
//! integrator = symplectic4    # 或 leapfrog
//! duration = 3.15576e9        # 模拟总时长（秒）
//! adaptive = true
//! softening = 1e6
//!
//! [body.Sun]
//! mass = 1.989e30; radius = 6.96e8
//! pos.x = 0; pos.y = 0; pos.z = 0
//! vel.x = 0; vel.y = 0; vel.z = 0
//!
//! [body.Earth]
//! mass = 5.972e24; radius = 6.371e6
//! pos.x = 1.496e11; pos.y = 0; pos.z = 0
//! vel.x = 0; vel.y = 29780; vel.z = 0
//! ```
//!
//! ## 运行时切换场景
//! 创建 `SceneRuntime` 后调用 `load_scene(&config)` 即可在不重置时间的
//! 前提下替换所有天体。也可用 `add_body` / `remove_body` 逐个体修改。
//!
//! `run_loop()` 支持"切换文件"机制：在主循环中定期检查一个文件，
//! 读取到新场景路径后自动加载并继续运行。可结合 `--switch-file` CLI 参数。

use std::collections::HashMap;
use std::fs;
use std::io::Write;

use crate::physics::{GravBody, GravitationalSystem};
use crate::Vec3;

/// 积分器类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IntegratorType {
    Leapfrog,
    Symplectic4,
}

/// 场景天体定义
#[derive(Debug, Clone)]
pub struct SceneBody {
    pub name: String,
    pub mass: f64,
    pub radius: f64,
    pub pos: Vec3,
    pub vel: Vec3,
}

/// 场景配置（不可变描述，用于构建或切换）
#[derive(Debug, Clone)]
pub struct SceneConfig {
    pub name: String,
    pub dt: f64,
    pub integrator: IntegratorType,
    pub duration: f64,
    pub adaptive: bool,
    pub softening: f64,
    pub csv: Option<String>,
    pub bodies: Vec<SceneBody>,
}

impl Default for SceneConfig {
    fn default() -> Self {
        SceneConfig {
            name: String::new(),
            dt: 1000.0,
            integrator: IntegratorType::Symplectic4,
            duration: 3.15576e10,
            adaptive: true,
            softening: 1e6,
            csv: None,
            bodies: Vec::new(),
        }
    }
}

// ---------------------------------------------------------------
// 解析工具
// ---------------------------------------------------------------

fn parse_vec3(section: &HashMap<String, String>, prefix: &str) -> Result<Vec3, String> {
    let get = |suffix: &str| -> Result<f64, String> {
        let key = format!("{prefix}.{suffix}");
        section
            .get(&key)
            .and_then(|v| v.parse().ok())
            .ok_or_else(|| format!("missing {prefix}{suffix}"))
    };
    Ok(Vec3::new(get("x")?, get("y")?, get("z")?))
}

fn parse_sections(content: &str) -> (Vec<String>, HashMap<String, HashMap<String, String>>) {
    let mut section_order: Vec<String> = Vec::new();
    let mut sections: HashMap<String, HashMap<String, String>> = HashMap::new();
    let mut current_section = String::new();

    for line in content.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with("//") {
            continue;
        }

        if let Some(rest) = line.strip_prefix('[').and_then(|s| s.strip_suffix(']')) {
            current_section = rest.to_string();
            if !sections.contains_key(&current_section) {
                section_order.push(current_section.clone());
            }
            sections.entry(current_section.clone()).or_default();
            continue;
        }

        for part in line.split(';') {
            let part = part.trim();
            if let Some(eq) = part.find('=') {
                let key = part[..eq].trim().to_string();
                let val = part[eq + 1..].trim().to_string();
                sections
                    .entry(current_section.clone())
                    .or_default()
                    .insert(key, val);
            }
        }
    }
    (section_order, sections)
}

// ---------------------------------------------------------------
// SceneConfig 实现
// ---------------------------------------------------------------

impl SceneConfig {
    /// 从 `.scene` 文件加载
    pub fn load(path: &str) -> Result<Self, String> {
        let content = fs::read_to_string(path)
            .map_err(|e| format!("Failed to read scene file '{path}': {e}"))?;
        Self::parse(&content)
    }

    /// 从字符串解析（测试用）
    pub fn parse(content: &str) -> Result<Self, String> {
        let (section_order, sections) = parse_sections(content);
        let mut config = SceneConfig::default();

        if let Some(scene) = sections.get("scene") {
            if let Some(v) = scene.get("name") {
                config.name = v.clone();
            }
            if let Some(v) = scene.get("dt").and_then(|v| v.parse().ok()) {
                config.dt = v;
            }
            if let Some(v) = scene.get("duration").and_then(|v| v.parse().ok()) {
                config.duration = v;
            }
            if let Some(v) = scene.get("softening").and_then(|v| v.parse().ok()) {
                config.softening = v;
            }
            if let Some(v) = scene.get("adaptive") {
                config.adaptive = matches!(v.as_str(), "true" | "yes" | "1");
            }
            if let Some(v) = scene.get("csv") {
                config.csv = Some(v.clone());
            }
            if let Some(v) = scene.get("integrator") {
                config.integrator = match v.as_str() {
                    "leapfrog" => IntegratorType::Leapfrog,
                    _ => IntegratorType::Symplectic4,
                };
            }
        }
        // 按文件顺序解析 [body.X] 段
        for sec_name in &section_order {
            if let Some(body_name) = sec_name.strip_prefix("body.") {
                let sec_data = &sections[sec_name];
                let mass = sec_data
                    .get("mass")
                    .and_then(|v| v.parse().ok())
                    .ok_or_else(|| format!("body '{body_name}' missing mass"))?;
                let radius = sec_data
                    .get("radius")
                    .and_then(|v| v.parse().ok())
                    .ok_or_else(|| format!("body '{body_name}' missing radius"))?;
                let pos = parse_vec3(sec_data, "pos")?;
                let vel = parse_vec3(sec_data, "vel")?;
                config.bodies.push(SceneBody {
                    name: body_name.to_string(),
                    mass,
                    radius,
                    pos,
                    vel,
                });
            }
        }

        if config.bodies.is_empty() {
            return Err("Scene has no bodies defined".to_string());
        }
        Ok(config)
    }

    /// 构建一个初始引力系统
    pub fn build_system(&self) -> GravitationalSystem {
        let mut sys = GravitationalSystem::new(self.softening);
        for b in &self.bodies {
            sys.add_body(GravBody::new(&b.name, b.mass, b.radius, b.pos, b.vel));
        }
        sys
    }
}

// ---------------------------------------------------------------
// SceneRuntime — 可热切换的仿真运行时
// ---------------------------------------------------------------

/// 可热切换的仿真运行时
///
/// 与 `SceneConfig` 的 `run()` 方法不同，`SceneRuntime` 保持状态，
/// 支持在模拟不中断的情况下切换场景配置。
pub struct SceneRuntime {
    pub sys: GravitationalSystem,
    pub dt: f64,
    pub integrator: IntegratorType,
    pub adaptive: bool,
    pub softening: f64,
    /// 当前场景配置的引用（用于 CSV 输出列名等）
    config_name: String,
}

impl SceneRuntime {
    /// 从场景配置创建仿真运行时
    pub fn new(config: &SceneConfig) -> Self {
        SceneRuntime {
            sys: config.build_system(),
            dt: config.dt,
            integrator: config.integrator,
            adaptive: config.adaptive,
            softening: config.softening,
            config_name: config.name.clone(),
        }
    }

    /// **切换场景** — 替换所有天体，但保持当前模拟时间不变
    ///
    /// - 清空现有天体
    /// - 从新场景配置添加天体（位置/速度重置为场景初始值）
    /// - `sys.time` 保持不变（模拟时钟不中断）
    /// - 积分器参数也同时更新
    pub fn load_scene(&mut self, config: &SceneConfig) {
        self.dt = config.dt;
        self.integrator = config.integrator;
        self.adaptive = config.adaptive;
        self.softening = config.softening;
        self.config_name = config.name.clone();

        // 保留时间，替换所有体
        let current_time = self.sys.time;
        let mut new_sys = GravitationalSystem::new(self.softening);
        for b in &config.bodies {
            new_sys.add_body(GravBody::new(&b.name, b.mass, b.radius, b.pos, b.vel));
        }
        new_sys.time = current_time;
        self.sys = new_sys;
    }

    /// 运行时添加一个天体
    pub fn add_body(&mut self, body: SceneBody) {
        self.sys.add_body(GravBody::new(
            &body.name,
            body.mass,
            body.radius,
            body.pos,
            body.vel,
        ));
    }

    /// 运行时移除一个天体（按名称）
    pub fn remove_body(&mut self, name: &str) -> bool {
        let before = self.sys.bodies.len();
        self.sys.bodies.retain(|b| b.name != name);
        self.sys.bodies.len() != before
    }

    /// 当前天体数量
    pub fn body_count(&self) -> usize {
        self.sys.bodies.len()
    }

    /// 当前场景名
    pub fn scene_name(&self) -> &str {
        &self.config_name
    }

    // -----------------------------------------------------------
    // 单步推进
    // -----------------------------------------------------------

    /// 推进一个时间步
    pub fn step(&mut self) {
        let remaining = self.dt;
        if self.adaptive {
            self.sys.step_adaptive(remaining, remaining * 1e-9);
        } else if self.integrator == IntegratorType::Symplectic4 {
            self.sys.step_symplectic4(remaining);
        } else {
            self.sys.step_leapfrog(remaining);
        }
    }

    // -----------------------------------------------------------
    // 运行循环
    // -----------------------------------------------------------

    /// 运行仿真，可选 CSV 输出和切换文件监控
    ///
    /// - `duration`：最大模拟持续时间（秒）
    /// - `report_interval_steps`：每多少步输出一次 CSV 行
    /// - `csv_path`：CSV 输出路径（`None` 表示不输出）
    /// - `switch_file`：场景切换监控文件（`None` 表示不监控）
    ///
    /// 返回 CSV 内容摘要（行数）
    pub fn run_loop(
        &mut self,
        duration: f64,
        report_interval_steps: usize,
        csv_path: Option<&str>,
        switch_file: Option<&str>,
    ) -> Result<usize, String> {
        let end_time = self.sys.time + duration;
        let mut csv: Option<fs::File> = None;
        let mut csv_lines = 1; // header

        if let Some(path) = csv_path {
            let file = fs::File::create(path)
                .map_err(|e| format!("Failed to create CSV '{path}': {e}"))?;
            let mut file = file;
            self.write_csv_header(&mut file)?;
            self.write_csv_line(&mut file)?;
            csv = Some(file);
            csv_lines += 1;
        }

        let mut step_count: usize = 0;

        while self.sys.time < end_time {
            self.step();
            step_count += 1;

            // CSV 输出
            if let Some(ref mut file) = csv {
                if step_count % report_interval_steps == 0 {
                    self.write_csv_line(file)?;
                    csv_lines += 1;
                }
            }

            // 检查切换文件
            if let Some(sw) = switch_file {
                if let Ok(content) = fs::read_to_string(sw) {
                    let path = content.trim().to_string();
                    if !path.is_empty() {
                        let _ = fs::remove_file(sw);
                        eprintln!("[runtime] switching scene -> {path}");
                        match SceneConfig::load(&path) {
                            Ok(cfg) => {
                                self.load_scene(&cfg);
                                eprintln!("[runtime] switched to '{}'", cfg.name);
                            }
                            Err(e) => {
                                eprintln!("[runtime] switch failed: {e}");
                            }
                        }
                    }
                }
            }
        }

        // 写入最终状态
        if let Some(ref mut file) = csv {
            self.write_csv_line(file)?;
            csv_lines += 1;
            eprintln!("[runtime] done — {csv_lines} CSV lines written");
        }

        Ok(csv_lines)
    }

    // -----------------------------------------------------------
    // CSV 辅助
    // -----------------------------------------------------------

    fn column_names(&self) -> Vec<String> {
        let mut cols = Vec::new();
        for b in &self.sys.bodies {
            for suffix in &["_x", "_y", "_z", "_vx", "_vy", "_vz"] {
                cols.push(format!("{}{}", b.name, suffix));
            }
        }
        cols.push("energy".to_string());
        cols.push("min_dist".to_string());
        cols
    }

    fn write_csv_header(&self, w: &mut impl Write) -> Result<(), String> {
        writeln!(w, "time,{}", self.column_names().join(","))
            .map_err(|e| format!("CSV write error: {e}"))
    }

    fn write_csv_line(&self, w: &mut impl Write) -> Result<(), String> {
        use std::fmt::Write as FmtWrite;
        let mut line = String::new();
        write!(line, "{:.6e}", self.sys.time).unwrap();
        for b in &self.sys.bodies {
            write!(
                line,
                ",{:.6e},{:.6e},{:.6e},{:.6e},{:.6e},{:.6e}",
                b.position.x, b.position.y, b.position.z, b.velocity.x, b.velocity.y, b.velocity.z
            )
            .unwrap();
        }
        write!(
            line,
            ",{:.6e},{:.6e}",
            self.sys.total_energy(),
            self.sys.min_distance()
        )
        .unwrap();
        writeln!(w, "{line}").map_err(|e| format!("CSV write error: {e}"))
    }
}

// ---------------------------------------------------------------
// 内置示例场景（可直接在代码中使用）
// ---------------------------------------------------------------

pub const SCENE_SOLAR_SYSTEM: &str = include_str!("../../scenes/solar_system.scene");
pub const SCENE_FIGURE8: &str = include_str!("../../scenes/figure8.scene");
pub const SCENE_THREE_BODY: &str = include_str!("../../scenes/three_body.scene");

// ---------------------------------------------------------------
// 测试
// ---------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> SceneConfig {
        SceneConfig::parse(SCENE_THREE_BODY).unwrap()
    }

    #[test]
    fn test_scene_load_solar_system() {
        let cfg = SceneConfig::parse(SCENE_SOLAR_SYSTEM).unwrap();
        assert!(cfg.name.contains("Solar"));
        assert_eq!(cfg.bodies.len(), 5);
    }

    #[test]
    fn test_scene_load_figure8() {
        let cfg = SceneConfig::parse(SCENE_FIGURE8).unwrap();
        assert_eq!(cfg.bodies.len(), 3);
        assert!(!cfg.adaptive);
    }

    #[test]
    fn test_scene_load_three_body() {
        let cfg = test_config();
        assert_eq!(cfg.bodies.len(), 3);
    }

    #[test]
    fn test_scene_missing_body() {
        let result = SceneConfig::parse("[scene]\ndt = 1\n");
        assert!(result.is_err());
    }

    #[test]
    fn test_scene_build_system() {
        let cfg = test_config();
        let sys = cfg.build_system();
        assert_eq!(sys.bodies.len(), 3);
    }

    #[test]
    fn test_runtime_new() {
        let cfg = test_config();
        let rt = SceneRuntime::new(&cfg);
        assert_eq!(rt.body_count(), 3);
        assert!((rt.sys.time - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_runtime_step() {
        let cfg = test_config();
        let mut rt = SceneRuntime::new(&cfg);
        rt.step();
        assert!(rt.sys.time > 0.0);
    }

    #[test]
    fn test_runtime_add_body() {
        let cfg = test_config();
        let mut rt = SceneRuntime::new(&cfg);
        rt.add_body(SceneBody {
            name: "Test".into(),
            mass: 1.0,
            radius: 0.1,
            pos: Vec3::new(1e12, 0.0, 0.0),
            vel: Vec3::zero(),
        });
        assert_eq!(rt.body_count(), 4);
    }

    #[test]
    fn test_runtime_remove_body() {
        let cfg = test_config();
        let mut rt = SceneRuntime::new(&cfg);
        assert!(rt.remove_body("Inner"));
        assert_eq!(rt.body_count(), 2);
        assert!(!rt.remove_body("NonExistent"));
    }

    #[test]
    fn test_runtime_load_scene_preserves_time() {
        let mut rt = SceneRuntime::new(&test_config());
        // 跑几步积累时间
        for _ in 0..10 {
            rt.step();
        }
        let t_before = rt.sys.time;
        assert!(t_before > 0.0);

        // 切换场景（图-8）
        let fig8 = SceneConfig::parse(SCENE_FIGURE8).unwrap();
        rt.load_scene(&fig8);

        // 时间不变
        assert!((rt.sys.time - t_before).abs() < 1e-12);
        // 但天体替换了
        assert_eq!(rt.body_count(), 3);
        assert_eq!(rt.sys.bodies[0].name, "A");
    }

    #[test]
    fn test_runtime_csv_output() {
        let cfg = test_config();
        let mut rt = SceneRuntime::new(&cfg);
        let csv_path = "./test_scene_runtime_csv.csv";
        // 只跑几步
        let prev_dt = rt.dt;
        rt.dt = 1000.0; // small steps for few iterations

        // Quick run with just a few steps
        // Manually create CSV output
        let mut file = fs::File::create(csv_path).unwrap();
        rt.write_csv_header(&mut file).unwrap();
        rt.write_csv_line(&mut file).unwrap();
        for _ in 0..5 {
            rt.step();
            rt.write_csv_line(&mut file).unwrap();
        }
        drop(file);

        let content = fs::read_to_string(csv_path).unwrap();
        assert!(content.starts_with("time,"));
        assert!(content.contains("Star_x"));
        let lines: Vec<&str> = content.lines().collect();
        assert_eq!(lines.len(), 7); // header + 1 initial + 5 steps
        let _ = fs::remove_file(csv_path);
        rt.dt = prev_dt;
    }

    #[test]
    fn test_runtime_run_loop_csv() {
        let cfg = test_config();
        let mut rt = SceneRuntime::new(&cfg);
        // 限制步数：用很小的 dt 跑很短时间
        rt.dt = 1000.0;
        rt.sys.time = 0.0;

        let csv_path = "./test_scene_runtime_loop.csv";
        let lines = rt.run_loop(5000.0, 1, Some(csv_path), None).unwrap();
        assert!(lines >= 2);

        let content = fs::read_to_string(csv_path).unwrap();
        assert!(content.len() > 50);
        let _ = fs::remove_file(csv_path);
    }

    #[test]
    fn test_runtime_switch_scene_no_csv() {
        // 测试切换文件机制
        let switch_path = "./test_scene_switch.scene_switch";

        // 先删除旧的切换文件
        let _ = fs::remove_file(switch_path);

        // 启动三体场景
        let mut rt = SceneRuntime::new(&test_config());
        rt.dt = 1000.0;
        rt.sys.time = 0.0;

        // 跑 10 步
        for _ in 0..10 {
            rt.step();
        }
        let t_before_switch = rt.sys.time;

        // 写入切换文件（临时写入一个有效场景）
        let fig8_path = "./test_scene_fig8_for_switch.scene";
        fs::write(fig8_path, SCENE_FIGURE8).unwrap();
        fs::write(switch_path, fig8_path).unwrap();

        // 再跑几步，其中一个步会检测到切换文件
        for _ in 0..20 {
            if let Ok(content) = fs::read_to_string(switch_path) {
                let path = content.trim().to_string();
                if !path.is_empty() {
                    let _ = fs::remove_file(switch_path);
                    if let Ok(cfg) = SceneConfig::load(&path) {
                        rt.load_scene(&cfg);
                    }
                }
            }
            rt.step();
        }

        // 切换后时间应继续推进
        assert!(rt.sys.time > t_before_switch);
        // 天体应为图-8 的 3 个体
        assert_eq!(rt.body_count(), 3);
        assert_eq!(rt.sys.bodies[0].name, "A");
        assert_eq!(rt.scene_name(), "Chenciner-Montgomery Figure-8");

        let _ = fs::remove_file(fig8_path);
    }
}
