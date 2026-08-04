//! 3D 渲染辅助 — 在 macroquad 之上封装太空可视化原语
//!
//! 提供轨道相机、行星绘制、姿态 Gizmo、轨迹线等功能。

use macroquad::camera::{set_camera, set_default_camera, Camera3D};
use macroquad::color::Color;
use macroquad::input::{
    is_key_pressed, is_mouse_button_down, is_mouse_button_pressed, is_mouse_button_released,
    mouse_position, mouse_wheel, KeyCode, MouseButton,
};
use macroquad::math::{Quat, Vec3};
use macroquad::models::{draw_line_3d, draw_sphere};
use macroquad::shapes::{draw_line, draw_rectangle};
use macroquad::text::{draw_text, draw_text_ex, load_ttf_font, Font, TextParams};
use macroquad::window::{screen_height, screen_width};
use std::sync::OnceLock;

// =====================================================================
// 自定义字体
// =====================================================================

static CUSTOM_FONT: OnceLock<Font> = OnceLock::new();

/// NASA Eyes 字体：Metropolis（抓取自 eyes.nasa.gov 公开 CDN，woff→ttf 转换）
pub const FONT_PATH: &str = "assets/fonts/Metropolis-Light.ttf";

/// 加载自定义 TTF 字体（在 async main 中调用一次）
pub async fn load_custom_font(path: &str) {
    match load_ttf_font(path).await {
        Ok(font) => {
            let _ = CUSTOM_FONT.set(font);
            eprintln!("  Custom font loaded: {}", path);
        }
        Err(e) => {
            eprintln!("  WARNING: failed to load font '{}': {}", path, e);
        }
    }
}

/// 绘制文本（使用自定义字体，若未加载则回退到 macroquad 默认字体）
pub fn text(text_str: impl AsRef<str>, x: f32, y: f32, font_size: f32, color: Color) {
    if let Some(font) = CUSTOM_FONT.get() {
        draw_text_ex(
            text_str.as_ref(),
            x,
            y,
            TextParams {
                font: Some(font),
                font_size: font_size as u16,
                color,
                ..Default::default()
            },
        );
    } else {
        draw_text(text_str.as_ref(), x, y, font_size, color);
    }
}

// =====================================================================
// 滑动轨迹窗口（环形缓冲区）
// =====================================================================

/// 轨迹环形缓冲区，仅保留最近 N 个点
pub struct Trail {
    points: Vec<macroquad::math::Vec3>,
    cursor: usize,
    full: bool,
}

impl Trail {
    pub fn new(length: usize) -> Self {
        Self {
            points: vec![macroquad::math::Vec3::ZERO; length],
            cursor: 0,
            full: false,
        }
    }

    pub fn push(&mut self, pos: macroquad::math::Vec3) {
        let len = self.points.len();
        if len == 0 {
            return;
        }
        self.points[self.cursor] = pos;
        self.cursor = (self.cursor + 1) % len;
        if self.cursor == 0 {
            self.full = true;
        }
    }

    /// 返回按时间顺序排列的有效点
    pub fn points(&self) -> Vec<macroquad::math::Vec3> {
        if self.full {
            // cursor 指向下一个要写的位置，所以 [cursor..] 是最老的，[..cursor] 是最新的
            let (newest, oldest) = self.points.split_at(self.cursor);
            [oldest, newest].concat()
        } else {
            self.points[..self.cursor].to_vec()
        }
    }
}

// =====================================================================
// UI 缩放：以 2560×1440 为参考分辨率
// =====================================================================

/// 返回 UI 缩放系数，基于当前窗口尺寸自动适配
pub fn ui_scale() -> f32 {
    let sx = screen_width() / 1920.0;
    let sy = screen_height() / 1080.0;
    sx.min(sy).clamp(0.3, 4.0)
}

// =====================================================================
// 轨道相机
// =====================================================================

/// 可拖拽/缩放的三维轨道相机
pub struct OrbitalCamera {
    /// 目标点（相机始终看向此处）
    pub target: Vec3,
    /// 相机距目标的距离（含平滑）
    pub distance: f32,
    /// 水平旋转角 (rad)
    pub azimuth: f32,
    /// 俯仰角 (rad)，限制在 (-π/2+ε, π/2-ε)
    pub elevation: f32,
    /// 垂直视野 (rad)
    pub fovy: f32,
    /// 鼠标拖拽灵敏度
    pub sensitivity: f32,
    /// 缩放灵敏度
    pub zoom_sensitivity: f32,
    /// 平滑系数（0~1，每帧趋近目标的比例，0.5 ≈ 2-帧 lerp）
    pub zoom_smooth_factor: f32,
    /// 距离限制
    pub min_distance: f32,
    pub max_distance: f32,
    // 鼠标拖拽状态（手动跟踪 delta）
    prev_mouse: Option<(f32, f32)>,
    // 平滑缩放的目标距离
    target_distance: f32,
}

impl OrbitalCamera {
    pub fn new(target: Vec3, distance: f32) -> Self {
        Self {
            target,
            distance,
            target_distance: distance,
            azimuth: 0.0,
            elevation: 0.4,
            fovy: std::f32::consts::FRAC_PI_4,
            sensitivity: 0.005,
            zoom_sensitivity: 0.1,
            zoom_smooth_factor: 0.15,
            min_distance: 1000.0,
            max_distance: 1.0e12,
            prev_mouse: None,
        }
    }

    /// 更新相机状态（鼠标拖拽旋转 + 滚轮缩放）
    pub fn update(&mut self) {
        if is_mouse_button_down(MouseButton::Left) {
            let (mx, my) = mouse_position();
            if let Some((px, py)) = self.prev_mouse {
                let dx = mx - px;
                let dy = my - py;
                self.azimuth -= dx * self.sensitivity;
                self.elevation = (self.elevation + dy * self.sensitivity).clamp(-1.5, 1.5);
            }
            self.prev_mouse = Some((mx, my));
        } else {
            self.prev_mouse = None;
        }

        let (_dx, dy) = mouse_wheel();
        if dy != 0.0 {
            // 归一化：不同平台 dy 值不同（Windows WHEEL_DELTA=120, GLFW=±1）
            // 用 signum 取方向忽略幅值，确保每格滚轮固定缩放 zoom_sensitivity
            let dir = dy.signum();
            self.target_distance *= (-dir * self.zoom_sensitivity).exp();
            self.target_distance = self
                .target_distance
                .clamp(self.min_distance, self.max_distance);
        }

        // 平滑趋近目标距离（每帧 15%，约 15 帧到达 90%）
        self.distance += (self.target_distance - self.distance) * self.zoom_smooth_factor;
    }

    /// 返回 Camera3D 供 macroquad 使用
    ///
    /// 自动根据 distance 设置近/远裁剪面，避免地球被 far plane 截断。
    /// 使用保守的 far/near 比 (5000) 而不是默认的 100000，保证 24-bit depth buffer 精度。
    pub fn get_camera3d(&self) -> Camera3D {
        let eye = self.eye_position();
        let d = self.distance.max(1.0);
        let z_near = (d * 0.001).max(0.01);
        // 包裹对象的最大距离 ≈ distance + 2×物体半径（地球 ~d*0.67）
        // 用 5× 留余量
        let z_far = (z_near * 5000.0).max(d * 5.0);
        Camera3D {
            position: eye,
            target: self.target,
            up: Vec3::Y,
            fovy: self.fovy,
            z_near,
            z_far,
            ..Default::default()
        }
    }

    /// 计算相机位置
    pub fn eye_position(&self) -> Vec3 {
        let x = self.distance * self.elevation.cos() * self.azimuth.sin();
        let y = self.distance * self.elevation.sin();
        let z = self.distance * self.elevation.cos() * self.azimuth.cos();
        self.target + Vec3::new(x, y, z)
    }

    /// 激活此相机
    pub fn set(&self) {
        set_camera(&self.get_camera3d());
    }

    /// 切换到默认相机（2D UI）
    pub fn set_default() {
        set_default_camera();
    }

    /// 正交投影：3D 世界坐标 → 2D 屏幕像素
    ///
    /// 将任意 3D 点投影到相机视角的 2D 屏幕上。
    /// 旋转相机 = 切换 2D 剖面。
    pub fn project_2d(&self, world: Vec3, sw: f32, sh: f32) -> (f32, f32) {
        let eye = self.eye_position();
        let fwd = (self.target - eye).normalize();

        // 防万向锁：视线平行于 Y 轴时用 X 作为右方向
        let world_up = Vec3::Y;
        let right = if fwd.dot(world_up).abs() > 0.999 {
            Vec3::X
        } else {
            fwd.cross(world_up).normalize()
        };
        let up = right.cross(fwd).normalize();

        let delta = world - self.target;
        let sx = delta.dot(right);
        let sy = delta.dot(up);

        // distance 控制缩放：大距离 = 小物体 = 更广视野
        let scale = (sw.min(sh) * 0.35) / self.distance.max(1.0);

        (sw / 2.0 + sx * scale, sh / 2.0 - sy * scale)
    }

    /// 世界空间长度 → 屏幕像素长度
    pub fn len_to_px(&self, world_len: f32, sw: f32, sh: f32) -> f32 {
        let scale = (sw.min(sh) * 0.35) / self.distance.max(1.0);
        world_len * scale
    }
}

// =====================================================================
// 颜色调色板
// =====================================================================

// NASA Eyes 设计令牌（抓取 app.css 的 CSS 变量，2026-08）
// 天体色：--mercury:#9768ac --venus:#b07919 --earth:#09c --mars:#9a4e19
//         --saturn:#d5c187 --uranus:#68ccda --neptune:#708ce3 --moon:#b6acac
//         --sun:#f7f4df --spacecraft:#cd9745 --asteroid:#806262
pub const COLOR_SUN: Color = Color::new(0.97, 0.96, 0.87, 1.0); // #f7f4df 暖白
pub const COLOR_MERCURY: Color = Color::new(0.59, 0.41, 0.67, 1.0); // #9768ac
pub const COLOR_VENUS: Color = Color::new(0.69, 0.47, 0.10, 1.0); // #b07919
pub const COLOR_EARTH: Color = Color::new(0.0, 0.6, 0.8, 1.0); // #09c 天蓝
pub const COLOR_MARS: Color = Color::new(0.60, 0.31, 0.10, 1.0); // #9a4e19
pub const COLOR_JUPITER: Color = Color::new(0.80, 0.70, 0.53, 1.0); // 介于 sandMed
pub const COLOR_SATURN: Color = Color::new(0.84, 0.76, 0.53, 1.0); // #d5c187
pub const COLOR_URANUS: Color = Color::new(0.41, 0.80, 0.85, 1.0); // #68ccda
pub const COLOR_NEPTUNE: Color = Color::new(0.44, 0.55, 0.89, 1.0); // #708ce3
pub const COLOR_MOON: Color = Color::new(0.71, 0.67, 0.67, 1.0); // #b6acac
pub const COLOR_SHIP: Color = Color::new(0.80, 0.59, 0.27, 1.0); // #cd9745 航天器金
pub const COLOR_PATH: Color = Color::new(0.3, 0.8, 1.0, 0.6);
pub const COLOR_GIZMO_X: Color = Color::new(1.0, 0.2, 0.2, 1.0);
pub const COLOR_GIZMO_Y: Color = Color::new(0.2, 1.0, 0.2, 1.0);
pub const COLOR_GIZMO_Z: Color = Color::new(0.2, 0.2, 1.0, 1.0);
pub const COLOR_TRAJECTORY: Color = Color::new(0.3, 0.8, 1.0, 0.8);
pub const COLOR_PREDICTION: Color = Color::new(1.0, 0.65, 0.1, 0.5); // 橙金色，与轨迹青蓝区分
pub const COLOR_GRID: Color = Color::new(0.25, 0.30, 0.40, 0.55);
pub const COLOR_GRID_AXIS: Color = Color::new(0.45, 0.55, 0.70, 0.75);
pub const COLOR_GROUND: Color = Color::new(0.3, 0.3, 0.4, 1.0);

// =====================================================================
// 绘制原语
// =====================================================================

/// 画行星（球体）
pub fn draw_planet(pos: Vec3, radius: f32, color: Color) {
    draw_sphere(pos, radius, None, color);
}

/// 画姿态 Gizmo（三个颜色轴表示飞船朝向）
///
/// 红=前方(X), 绿=上方(Y), 蓝=右方(Z)
pub fn draw_gizmo(pos: Vec3, orientation: &Quat, scale: f32) {
    let axes = [
        (Vec3::X, COLOR_GIZMO_X),
        (Vec3::Y, COLOR_GIZMO_Y),
        (Vec3::Z, COLOR_GIZMO_Z),
    ];

    for (local_dir, color) in &axes {
        let world_dir = *orientation * *local_dir;
        let end = pos + world_dir * scale;
        draw_line_3d(pos, end, *color);
    }
}

/// 画轨迹线（通过一系列点）
pub fn draw_path(points: &[Vec3], color: Color) {
    for window in points.windows(2) {
        draw_line_3d(window[0], window[1], color);
    }
}

/// 画速度向量箭头
pub fn draw_velocity_arrow(pos: Vec3, vel: Vec3, scale: f32) {
    let dir = vel.normalize_or_zero();
    let end = pos + dir * scale;
    draw_line_3d(pos, end, COLOR_GIZMO_Y);

    if dir.length_squared() > 0.001 {
        let head_len = scale * 0.15;
        let perp = if dir.x.abs() < 0.9 {
            dir.cross(Vec3::X).normalize()
        } else {
            dir.cross(Vec3::Y).normalize()
        };
        let spread = 0.3;
        let head1 = end - dir * head_len + perp * head_len * spread;
        let head2 = end - dir * head_len - perp * head_len * spread;
        draw_line_3d(end, head1, COLOR_GIZMO_Y);
        draw_line_3d(end, head2, COLOR_GIZMO_Y);
    }
}

// =====================================================================
// 轨道预测（RK4 二体数值传播）
// =====================================================================

/// 使用 RK4 数值积分预测轨道（仅中心引力，忽略摄动）
///
/// - `pos`, `vel`: 当前状态（惯性系，Y-up）
/// - `mu`: 中心天体标准引力参数 (m³/s²)
/// - `perturbers`: 摄动天体列表 (位置, 质量)，如月球
/// - `duration`: 预测时长 (s)
/// - `num_points`: 采样点数（决定线条平滑度）
/// - `earth_radius`: 若 > 0，轨迹进入地表以下时截断（避免穿透地球）
/// - 返回惯性系中的位置序列
pub fn predict_trajectory(
    pos: deepspace::Vec3,
    vel: deepspace::Vec3,
    mu: f64,
    perturbers: &[(deepspace::Vec3, f64)],
    duration: f64,
    num_points: usize,
    earth_radius: f64,
) -> Vec<deepspace::Vec3> {
    let n = num_points.max(2);
    let dt = duration / n as f64;
    let mut s = [pos.x, pos.y, pos.z, vel.x, vel.y, vel.z];
    let mut points = Vec::with_capacity(n);
    points.push(pos);

    for _ in 1..n {
        s = rk4_nbody(&s, mu, perturbers, dt);
        let p = deepspace::Vec3::new(s[0], s[1], s[2]);
        // 截断：进入地表以下则停止
        if earth_radius > 0.0 && p.length() < earth_radius {
            break;
        }
        points.push(p);
    }
    points
}

/// RK4 单步（N体引力：中心天体 + 摄动体）
fn rk4_nbody(s: &[f64; 6], mu: f64, perturbers: &[(deepspace::Vec3, f64)], dt: f64) -> [f64; 6] {
    let deriv = |y: &[f64; 6]| -> [f64; 6] {
        let r2 = y[0] * y[0] + y[1] * y[1] + y[2] * y[2];
        let mut ax = 0.0;
        let mut ay = 0.0;
        let mut az = 0.0;
        if r2 > 0.0 {
            let a = -mu / (r2 * r2.sqrt());
            ax = y[0] * a;
            ay = y[1] * a;
            az = y[2] * a;
        }
        // 摄动体引力
        let g = deepspace::G;
        for (ppos, pmass) in perturbers {
            let dx = ppos.x - y[0];
            let dy = ppos.y - y[1];
            let dz = ppos.z - y[2];
            let d2 = dx * dx + dy * dy + dz * dz;
            if d2 > 0.0 {
                let acc = g * pmass / (d2 * d2.sqrt());
                ax += dx * acc;
                ay += dy * acc;
                az += dz * acc;
            }
        }
        [y[3], y[4], y[5], ax, ay, az]
    };

    let k1 = deriv(s);
    let mut t = [0.0; 6];
    for i in 0..6 {
        t[i] = s[i] + 0.5 * dt * k1[i];
    }
    let k2 = deriv(&t);

    for i in 0..6 {
        t[i] = s[i] + 0.5 * dt * k2[i];
    }
    let k3 = deriv(&t);

    for i in 0..6 {
        t[i] = s[i] + dt * k3[i];
    }
    let k4 = deriv(&t);

    let mut out = [0.0; 6];
    for i in 0..6 {
        out[i] = s[i] + dt * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;
    }
    out
}

/// 画预测轨道线（虚线风格 — 隔段绘制）
pub fn draw_predicted_path(points: &[deepspace::Vec3], color: Color) {
    if points.len() < 2 {
        return;
    }
    // 虚线：隔一段画一段
    for i in (0..points.len() - 1).step_by(2) {
        let p0 = Vec3::new(points[i].x as f32, points[i].y as f32, points[i].z as f32);
        let p1 = Vec3::new(
            points[i + 1].x as f32,
            points[i + 1].y as f32,
            points[i + 1].z as f32,
        );
        draw_line_3d(p0, p1, color);
    }
}

// =====================================================================
// 空间参考网格
// =====================================================================

/// 绘制空间参考网格 — 同心赤道环 + 径向辐条 + 轴参考线
///
/// 以 `center` 为中心，以 `earth_radius` 为基本单位绘制多层同心圆环，
/// 提供空间距离和方位的视觉参考。
pub fn draw_spatial_grid(center: Vec3, earth_radius: f32) {
    const RINGS: [f32; 5] = [1.5, 2.0, 3.0, 5.0, 10.0];
    const SEGMENTS: u32 = 48;
    const SPOKES: u32 = 12;

    // 1. 同心赤道环（XZ 平面）
    for &mult in &RINGS {
        draw_circle_3d(center, earth_radius * mult, SEGMENTS, COLOR_GRID);
    }

    // 2. 径向辐条（赤道面）
    let max_r = earth_radius * 10.0;
    for i in 0..SPOKES {
        let angle = (i as f32 / SPOKES as f32) * std::f32::consts::TAU;
        let dir = Vec3::new(angle.cos(), 0.0, angle.sin());
        draw_line_3d(center, center + dir * max_r, COLOR_GRID);
    }

    // 3. 坐标轴线（稍亮，便于辨认方向）
    let axis_len = earth_radius * 12.0;
    // X 轴（红调）
    let cx = COLOR_GRID_AXIS;
    draw_line_3d(center - Vec3::X * axis_len, center + Vec3::X * axis_len, cx);
    // Y 轴（绿调）
    draw_line_3d(center - Vec3::Y * axis_len, center + Vec3::Y * axis_len, cx);
    // Z 轴（蓝调）
    draw_line_3d(center - Vec3::Z * axis_len, center + Vec3::Z * axis_len, cx);
}

// =====================================================================
// 2D 姿态指示器（屏幕空间 HUD）
// =====================================================================

/// 绘制 2D 三轴姿态指示器
///
/// 将火箭本体三轴（红=X前, 绿=Y上, 蓝=Z右）投影到相机屏幕平面，
/// 在固定屏幕位置绘制，不受缩放影响。旋转相机时轴方向随之变化。
///
/// - `cx`, `cy`: 指示器中心在屏幕上的像素坐标
/// - `size`: 半轴长（像素）
/// - `orientation`: 火箭姿态四元数（本体→世界）
/// - `cam_eye`, `cam_target`: 相机视点，用于计算投影平面
pub fn draw_attitude_indicator_2d(
    cx: f32,
    cy: f32,
    size: f32,
    orientation: &Quat,
    cam_eye: Vec3,
    cam_target: Vec3,
) {
    let s = ui_scale();
    let cam_fwd = (cam_target - cam_eye).normalize();
    // 用世界 Y 作为参考上方向，计算相机右/上向量
    let cam_right = cam_fwd.cross(Vec3::Y).normalize();
    let cam_up = cam_right.cross(cam_fwd).normalize();

    let axes = [
        (Vec3::X, COLOR_GIZMO_X, "F"), // 前方 X → 红
        (Vec3::Y, COLOR_GIZMO_Y, "U"), // 上方 Y → 绿
        (Vec3::Z, COLOR_GIZMO_Z, "R"), // 右方 Z → 蓝
    ];

    for (local_dir, color, label) in &axes {
        let world_dir = *orientation * *local_dir;

        // 投影到屏幕平面
        let sx = world_dir.dot(cam_right);
        let sy = world_dir.dot(cam_up);
        let len = (sx * sx + sy * sy).sqrt();
        if len < 0.001 {
            continue;
        }

        // 深度测试：指向相机 → 不透明；背向相机 → 半透明
        let depth = world_dir.dot(cam_fwd);
        let alpha = if depth > 0.0 { 1.0 } else { 0.2 };

        let mut c = *color;
        c.a *= alpha;

        let nx = sx / len;
        let ny = sy / len;
        let ex = cx + nx * size;
        let ey = cy + ny * size;

        draw_line(cx, cy, ex, ey, 2.0, c);

        // 轴标签
        text(label, ex - 5.0 * s, ey - 5.0 * s, 18.0 * s, c);
    }
}

// =====================================================================
// 2D 正交投影渲染 — 整个场景为 2D HUD
// =====================================================================

/// 画 2D 圆环（自适应分段，大圆用更多段避免锯齿）
pub fn draw_circle_2d(cx: f32, cy: f32, radius: f32, color: Color) {
    if radius < 0.5 {
        return;
    }
    // 像素半径越大段数越多，保证视觉平滑。放大后可达 256 段
    let segs = (12.0 + radius * 0.3).min(256.0) as u32;
    let step = std::f32::consts::TAU / segs as f32;
    for i in 0..segs {
        let a0 = i as f32 * step;
        let a1 = (i + 1) as f32 * step;
        let (s0, c0) = a0.sin_cos();
        let (s1, c1) = a1.sin_cos();
        draw_line(
            cx + c0 * radius,
            cy + s0 * radius,
            cx + c1 * radius,
            cy + s1 * radius,
            1.0,
            color,
        );
    }
}

/// 投影绘制地球（2D 圆 + 中心十字）
pub fn draw_earth_2d(camera: &OrbitalCamera, earth_radius: f32, sw: f32, sh: f32) {
    let (cx, cy) = camera.project_2d(Vec3::ZERO, sw, sh);
    let r = camera.len_to_px(earth_radius, sw, sh);
    // 地球轮廓
    draw_circle_2d(cx, cy, r, COLOR_EARTH);
    // 地心十字
    let cross = 6.0_f32.max(r * 0.04);
    draw_line(cx - cross, cy, cx + cross, cy, 1.0, COLOR_EARTH);
    draw_line(cx, cy - cross, cx, cy + cross, 1.0, COLOR_EARTH);
}

/// 投影绘制经纬线网格 + 坐标轴
///
/// 纬线（平行圈）：在不同纬度上绕 Y 轴的水平圆
/// 经线（子午线）：从南极到北极的垂直弧
pub fn draw_grid_2d(camera: &OrbitalCamera, earth_radius: f32, sw: f32, sh: f32) {
    let s = ui_scale();
    const LATITUDES: [f32; 5] = [0.0, 30.0, 60.0, -30.0, -60.0];
    const MERIDIANS: u32 = 12;
    const CENTER: Vec3 = Vec3::ZERO;

    // 1. 纬线（平行圈）
    for &lat_deg in &LATITUDES {
        let lat = lat_deg.to_radians();
        let r = earth_radius * lat.cos(); // 平行圈半径
        let y = earth_radius * lat.sin(); // 平行圈高度
        let pixel_r = camera.len_to_px(r, sw, sh);
        let segs = (12.0 + pixel_r * 0.3).min(256.0) as u32;
        if segs < 4 {
            continue;
        }
        let step = std::f32::consts::TAU / segs as f32;
        let mut prev = None;
        for i in 0..=segs {
            let a = ((i % segs) as f32) * step;
            let p3d = Vec3::new(a.cos() * r, y, a.sin() * r);
            let (x, y) = camera.project_2d(p3d, sw, sh);
            if let Some((px, py)) = prev {
                draw_line(px, py, x, y, 1.0, COLOR_GRID);
            }
            prev = Some((x, y));
        }
    }

    // 2. 经线（子午线）
    for i in 0..MERIDIANS {
        let lon = (i as f32 / MERIDIANS as f32) * std::f32::consts::TAU;
        let segs = 64u32;
        let step = std::f32::consts::PI / segs as f32;
        let mut prev = None;
        for j in 0..=segs {
            let theta = (j as f32) * step - std::f32::consts::FRAC_PI_2; // -90° → 90°
            let p3d = Vec3::new(
                earth_radius * theta.cos() * lon.cos(),
                earth_radius * theta.sin(),
                earth_radius * theta.cos() * lon.sin(),
            );
            let (x, y) = camera.project_2d(p3d, sw, sh);
            if let Some((px, py)) = prev {
                draw_line(px, py, x, y, 1.0, COLOR_GRID);
            }
            prev = Some((x, y));
        }
    }

    // 3. 坐标轴线
    let a_len = earth_radius * 1.8;
    let xp = camera.project_2d(CENTER + Vec3::X * a_len, sw, sh);
    let xn = camera.project_2d(CENTER - Vec3::X * a_len, sw, sh);
    draw_line(xn.0, xn.1, xp.0, xp.1, 1.5, COLOR_GRID_AXIS);

    let yp = camera.project_2d(CENTER + Vec3::Y * a_len, sw, sh);
    let yn = camera.project_2d(CENTER - Vec3::Y * a_len, sw, sh);
    draw_line(yn.0, yn.1, yp.0, yp.1, 1.5, COLOR_GRID_AXIS);

    let zp = camera.project_2d(CENTER + Vec3::Z * a_len, sw, sh);
    let zn = camera.project_2d(CENTER - Vec3::Z * a_len, sw, sh);
    draw_line(zn.0, zn.1, zp.0, zp.1, 1.5, COLOR_GRID_AXIS);

    // 轴标签
    text(
        "X",
        xp.0 + 5.0 * s,
        xp.1 - 5.0 * s,
        18.0 * s,
        COLOR_GRID_AXIS,
    );
    text(
        "Y",
        yp.0 + 5.0 * s,
        yp.1 - 5.0 * s,
        18.0 * s,
        COLOR_GRID_AXIS,
    );
    text(
        "Z",
        zp.0 + 5.0 * s,
        zp.1 - 5.0 * s,
        18.0 * s,
        COLOR_GRID_AXIS,
    );
}

/// 投影绘制 2D 轨迹线
pub fn draw_path_2d(camera: &OrbitalCamera, points: &[Vec3], sw: f32, sh: f32, color: Color) {
    for w in points.windows(2) {
        let (x1, y1) = camera.project_2d(w[0], sw, sh);
        let (x2, y2) = camera.project_2d(w[1], sw, sh);
        draw_line(x1, y1, x2, y2, 1.0, color);
    }
}

/// 投影绘制预测轨道（虚线：所有段都画，交替透明度）
pub fn draw_predicted_path_2d(
    camera: &OrbitalCamera,
    points: &[deepspace::Vec3],
    sw: f32,
    sh: f32,
    color: Color,
) {
    if points.len() < 2 {
        return;
    }
    // 画所有段，奇偶交替透明度实现虚线效果，不损失分辨率
    let mut c = color;
    for i in 0..points.len() - 1 {
        c.a = if i % 2 == 0 { color.a } else { color.a * 0.2 };
        let p0 = Vec3::new(points[i].x as f32, points[i].y as f32, points[i].z as f32);
        let p1 = Vec3::new(
            points[i + 1].x as f32,
            points[i + 1].y as f32,
            points[i + 1].z as f32,
        );
        let (x1, y1) = camera.project_2d(p0, sw, sh);
        let (x2, y2) = camera.project_2d(p1, sw, sh);
        draw_line(x1, y1, x2, y2, 1.0, c);
    }
}

/// 投影绘制火箭标记 + 速度方向箭头
pub fn draw_rocket_2d(
    camera: &OrbitalCamera,
    pos: Vec3,
    vel: Vec3,
    earth_radius: f32,
    sw: f32,
    sh: f32,
) {
    let (rx, ry) = camera.project_2d(pos, sw, sh);

    // 火箭位置圆点
    let marker_r = camera
        .len_to_px(2000.0_f32.max(earth_radius * 0.003), sw, sh)
        .max(2.5);
    draw_circle_2d(rx, ry, marker_r, COLOR_SHIP);

    // 速度方向箭头（纯 2D 屏幕空间，固定像素长度）
    let speed = vel.length();
    if speed > 1.0 {
        let vd = vel / speed;
        // 计算速度方向在屏幕上的投影
        let eye = camera.eye_position();
        let fwd = (camera.target - eye).normalize();
        let world_up = Vec3::Y;
        let right = if fwd.dot(world_up).abs() > 0.999 {
            Vec3::X
        } else {
            fwd.cross(world_up).normalize()
        };
        let up = right.cross(fwd).normalize();

        let sx = vd.dot(right);
        let sy = vd.dot(up); // 屏幕 Y 下方向
        let dlen = (sx * sx + sy * sy).sqrt().max(0.001);
        let nx = sx / dlen;
        let ny = sy / dlen;

        let arrow_len = 28.0_f32.max(marker_r * 3.0);
        let ax = rx + nx * arrow_len;
        let ay = ry - ny * arrow_len; // Y 翻转

        // 箭杆
        draw_line(rx, ry, ax, ay, 1.5, COLOR_GIZMO_Y);
        // 箭头
        let head = 7.0;
        draw_line(
            ax,
            ay,
            ax - nx * head + ny * head * 0.5,
            ay + ny * head + nx * head * 0.5,
            1.5,
            COLOR_GIZMO_Y,
        );
        draw_line(
            ax,
            ay,
            ax - nx * head - ny * head * 0.5,
            ay + ny * head - nx * head * 0.5,
            1.5,
            COLOR_GIZMO_Y,
        );
    }
}

// =====================================================================
// 3D 中的 2D 图形（传统 3D 渲染用，火箭-sim 改用上述 2D 投影函数）
// =====================================================================

/// 在 3D 空间中画 2D 圆环（XZ 平面，即水平环）
pub fn draw_circle_3d(center: Vec3, radius: f32, segments: u32, color: Color) {
    let step = std::f32::consts::TAU / segments as f32;
    for i in 0..segments {
        let a0 = i as f32 * step;
        let a1 = (i + 1) as f32 * step;
        let p0 = center + Vec3::new(a0.cos() * radius, 0.0, a0.sin() * radius);
        let p1 = center + Vec3::new(a1.cos() * radius, 0.0, a1.sin() * radius);
        draw_line_3d(p0, p1, color);
    }
}

/// 在 3D 空间中画 2D 垂直圆环（YZ 平面，子午线）
pub fn draw_meridian_3d(center: Vec3, radius: f32, segments: u32, color: Color) {
    let step = std::f32::consts::TAU / segments as f32;
    for i in 0..segments {
        let a0 = i as f32 * step;
        let a1 = (i + 1) as f32 * step;
        let p0 = center + Vec3::new(0.0, a0.sin() * radius, a0.cos() * radius);
        let p1 = center + Vec3::new(0.0, a1.sin() * radius, a1.cos() * radius);
        draw_line_3d(p0, p1, color);
    }
}

// =====================================================================
// 任务导航面板（HUD 右侧）— 阶段 + 任务双层显示
// =====================================================================

/// Artemis II 全任务阶段（粗粒度飞行阶段）
const ARTEMIS_PHASES: &[&str] = &[
    "PRE_LAUNCH",
    "LAUNCH",
    "ASCENT",
    "ORBIT",
    "TLI",
    "TRANSLUNAR",
    "LUNAR_FLYBY",
    "RETURN",
    "REENTRY",
    "SUCCESS",
];

/// Artemis II 具体任务里程碑（细粒度事件步骤）
/// 任务显示状态（由 3D 主循环计算后传入）
pub struct MissionDisplayState {
    /// 数据驱动阶段名（直接来自 MissionControl.phase_name）
    pub phase_name: String,
    /// 是否已完成
    pub complete: bool,
    /// 结果字符串（SUCCESS/FAILURE/ABORT/TIMEOUT）
    pub outcome: String,
}

/// 将数据驱动阶段名映射到 ARTEMIS_PHASES 索引（用于可视时间轴标记）
fn phase_to_idx(name: &str) -> Option<usize> {
    // 严格匹配数据驱动阶段名，不做时间/距离启发式
    match name {
        "PreLaunch" => Some(0),                                       // PRE_LAUNCH
        "Launch" => Some(1),                                          // LAUNCH
        "Ascent" | "MaxQ" => Some(2),                                 // ASCENT
        "Orbit" | "Staging" | "Coast" | "Circularization" => Some(3), // ORBIT
        "TEI" => Some(4),                                             // TLI
        "Translunar" | "MissionEvents" => Some(5),                    // TRANSLUNAR（去程统称）
        "Reentry" => Some(8),                                         // REENTRY
        "Success" | "Failure" | "Abort" => Some(9),                   // SUCCESS
        _ => None,                                                    // 不认识的新阶段名 → 不标记
    }
}

/// 绘制阶段面板（右侧上方）
pub fn draw_phase_panel(state: &MissionDisplayState, x: f32, y: f32) {
    let s = ui_scale();
    let line_h = 30.0 * s;
    let item_start_y = y + 32.0 * s;
    let panel_w = 260.0 * s;
    let n_phases = ARTEMIS_PHASES.len() as f32;
    let content_h = (item_start_y - y) + n_phases * line_h + 12.0 * s;

    // 半透明背景
    draw_rectangle(
        x - 8.0 * s,
        y - 28.0 * s,
        panel_w + 16.0 * s,
        content_h + 8.0 * s,
        Color::new(0.05, 0.05, 0.1, 0.75),
    );

    // 标题 + 当前实际阶段名
    text("PHASE", x, y, 24.0 * s, Color::new(0.7, 0.8, 1.0, 0.95));
    text(
        &state.phase_name,
        x + 56.0 * s,
        y,
        20.0 * s,
        Color::new(1.0, 0.9, 0.2, 1.0),
    );

    let current_idx = phase_to_idx(&state.phase_name);

    for (i, &name) in ARTEMIS_PHASES.iter().enumerate() {
        let py = item_start_y + i as f32 * line_h;

        let (icon, color) = if state.complete {
            ("\u{2713}", Color::new(0.4, 0.6, 0.6, 0.7))
        } else if current_idx.is_some_and(|c| i < c) {
            ("\u{2713}", Color::new(0.5, 0.7, 0.7, 0.7))
        } else if current_idx == Some(i) {
            ("\u{25B6}", Color::new(1.0, 0.9, 0.2, 1.0))
        } else {
            ("\u{25CB}", Color::new(0.5, 0.5, 0.6, 0.8))
        };

        text(icon, x, py, 20.0 * s, color);
        text(name, x + 24.0 * s, py, 20.0 * s, color);
    }

    if state.complete {
        let rc = match state.outcome.as_str() {
            "SUCCESS" => Color::new(0.2, 1.0, 0.3, 1.0),
            "FAILURE" => Color::new(1.0, 0.2, 0.2, 1.0),
            _ => Color::new(1.0, 0.8, 0.2, 1.0),
        };
        text(
            format!("OUTCOME: {}", state.outcome),
            x,
            item_start_y + n_phases * line_h + 6.0 * s,
            22.0 * s,
            rc,
        );
    }
}

// 绘制任务里程碑面板（细粒度，右侧下方）
// =====================================================================
// 后续阶段进度面板（数据驱动）
// =====================================================================

/// 绘制所有后续阶段转换面板
pub fn draw_remaining_phases_panel(phases: &[NextPhaseDisplay], x: f32, y: f32) {
    let s = ui_scale();
    let panel_w = 280.0 * s;
    let header_h = 40.0 * s;
    let phase_sep = 8.0 * s;

    // 计算总高度
    let mut total_h = header_h;
    for phase in phases {
        total_h += 28.0 * s; // 阶段标题行
        total_h += phase
            .conditions
            .iter()
            .map(|c| if c.is_boolean { 22.0 * s } else { 38.0 * s })
            .sum::<f32>();
        total_h += phase_sep;
    }
    total_h += 8.0 * s;

    // 半透明背景
    draw_rectangle(
        x - 8.0 * s,
        y - 26.0 * s,
        panel_w + 16.0 * s,
        total_h,
        Color::new(0.05, 0.05, 0.1, 0.75),
    );

    text(
        "UPCOMING PHASES",
        x,
        y,
        22.0 * s,
        Color::new(0.7, 0.8, 1.0, 0.95),
    );

    let mut cy = y + header_h;

    for phase in phases {
        // 阶段名 + 逻辑（ALL/ANY）
        let logic = if phase.require_all { "ALL" } else { "ANY" };
        text(
            &phase.next_phase,
            x + 4.0 * s,
            cy,
            20.0 * s,
            Color::new(1.0, 0.9, 0.2, 1.0),
        );
        text(
            logic,
            x + panel_w - 30.0 * s,
            cy,
            12.0 * s,
            Color::new(0.4, 0.5, 0.6, 0.6),
        );
        cy += 24.0 * s;

        // 分隔线
        draw_line(
            x,
            cy - 6.0 * s,
            x + panel_w,
            cy - 6.0 * s,
            1.0,
            Color::new(0.5, 0.5, 0.6, 0.3),
        );

        for cond in &phase.conditions {
            if cond.is_boolean {
                let (icon, color) = if cond.is_met {
                    ("\u{2713}", Color::new(0.3, 0.9, 0.3, 1.0))
                } else {
                    ("\u{25CB}", Color::new(0.6, 0.6, 0.7, 0.8))
                };
                text(icon, x + 4.0 * s, cy, 16.0 * s, color);
                text(&cond.label, x + 24.0 * s, cy, 16.0 * s, color);
                cy += 22.0 * s;
            } else {
                // 非布尔条件：标签 + 进度条
                let pct = (cond.progress * 100.0).min(99.9);
                let (icon, color) = if cond.is_met {
                    ("\u{2713}", Color::new(0.3, 0.9, 0.3, 1.0))
                } else {
                    ("\u{25B6}", Color::new(1.0, 0.9, 0.2, 1.0))
                };
                text(icon, x + 4.0 * s, cy, 16.0 * s, color);
                text(
                    format!(
                        "{} {:.0}/{:.0} [{:.0}%]",
                        cond.label, cond.current, cond.target, pct
                    ),
                    x + 24.0 * s,
                    cy,
                    14.0 * s,
                    color,
                );
                cy += 18.0 * s;

                // 进度条
                let bar_w = panel_w - 28.0 * s;
                let bar_h = 6.0 * s;
                draw_rectangle(
                    x + 24.0 * s,
                    cy,
                    bar_w,
                    bar_h,
                    Color::new(0.2, 0.2, 0.3, 0.8),
                );
                let fill = (bar_w * cond.progress as f32).min(bar_w).max(0.0);
                let bar_color = if cond.is_met {
                    Color::new(0.3, 0.9, 0.3, 0.9)
                } else {
                    Color::new(1.0, 0.9, 0.2, 0.9)
                };
                draw_rectangle(x + 24.0 * s, cy, fill, bar_h, bar_color);
                cy += 22.0 * s;
            }
        }

        cy += phase_sep;
    }
}

/// 下一阶段条件的 UI 显示数据
pub struct NextPhaseConditionDisplay {
    pub label: String,
    pub current: f64,
    pub target: f64,
    pub progress: f64,
    pub is_met: bool,
    pub is_boolean: bool,
}

/// 下一阶段进度 UI 显示数据
pub struct NextPhaseDisplay {
    pub next_phase: String,
    pub conditions: Vec<NextPhaseConditionDisplay>,
    pub require_all: bool,
}

/// 绘制下一阶段进度面板（右侧，任务面板下方）
pub fn draw_next_phase_panel(state: &NextPhaseDisplay, x: f32, y: f32) {
    let s = ui_scale();
    let panel_w = 190.0 * s;

    // 先计算总高度
    let header_h = 40.0 * s;
    let line_h = 24.0 * s;
    let cond_h: f32 = state
        .conditions
        .iter()
        .map(|c| if c.is_boolean { line_h } else { 44.0 * s })
        .sum();
    let content_h = header_h + 4.0 * s + cond_h + 8.0 * s;

    // 半透明背景
    draw_rectangle(
        x - 8.0 * s,
        y - 26.0 * s,
        panel_w + 16.0 * s,
        content_h + 8.0 * s,
        Color::new(0.05, 0.05, 0.1, 0.75),
    );

    // 标题
    text("NEXT", x, y, 22.0 * s, Color::new(0.7, 0.8, 1.0, 0.95));

    // 下一阶段名
    text(
        &state.next_phase,
        x + 4.0 * s,
        y + 24.0 * s,
        20.0 * s,
        Color::new(1.0, 0.9, 0.2, 1.0),
    );

    // 条件逻辑提示（ALL / ANY）
    let logic = if state.require_all { "ALL" } else { "ANY" };
    text(
        logic,
        x + panel_w - 30.0 * s,
        y + 24.0 * s,
        12.0 * s,
        Color::new(0.4, 0.5, 0.6, 0.6),
    );

    // 分隔线
    draw_line(
        x,
        y + 32.0 * s,
        x + panel_w,
        y + 32.0 * s,
        1.0,
        Color::new(0.5, 0.5, 0.6, 0.5),
    );

    let mut cy = y + 44.0 * s;
    let line_h = 24.0 * s;

    for cond in &state.conditions {
        if cond.is_boolean {
            // 布尔条件：显示标签 + 图标
            let (icon, color) = if cond.is_met {
                ("\u{2713}", Color::new(0.3, 0.9, 0.3, 1.0)) // ✓
            } else {
                ("\u{25CB}", Color::new(0.6, 0.6, 0.7, 0.8)) // ○
            };
            text(icon, x, cy, 18.0 * s, color);
            text(&cond.label, x + 20.0 * s, cy, 18.0 * s, color);
            cy += line_h;
        } else {
            // 数值条件：显示标签 + 数值 + 进度条
            let label_color = if cond.is_met {
                Color::new(0.3, 0.9, 0.3, 1.0)
            } else {
                Color::new(0.8, 0.8, 0.9, 0.9)
            };
            text(&cond.label, x, cy, 16.0 * s, label_color);

            // 数值 (current / target)
            let val_text = if cond.target < 10.0 {
                format!("{:.2}/{:.2}", cond.current, cond.target)
            } else {
                format!("{:.0}/{:.0}", cond.current, cond.target)
            };
            let pct = (cond.progress * 100.0).min(99.9) as u32;
            let info = format!("{}  {}%", val_text, pct);
            text(
                &info,
                x,
                cy + 16.0 * s,
                14.0 * s,
                Color::new(0.6, 0.7, 0.8, 0.9),
            );

            // 进度条背景
            let bar_y = cy + 28.0 * s;
            let bar_w = panel_w;
            let bar_h = 6.0 * s;
            draw_rectangle(x, bar_y, bar_w, bar_h, Color::new(0.2, 0.2, 0.3, 0.7));

            // 进度条填充
            let fill_w = (bar_w * cond.progress as f32).min(bar_w);
            let fill_color = if cond.is_met {
                Color::new(0.3, 0.9, 0.3, 0.8)
            } else {
                Color::new(0.4, 0.6, 0.9, 0.8)
            };
            draw_rectangle(x, bar_y, fill_w, bar_h, fill_color);

            cy += 44.0 * s;
        }
    }
}

// =====================================================================
// 坐标转换
// =====================================================================

/// 将 deepspace::Vec3 (f64) 转换为 macroquad Vec3 (f32)
pub fn to_mvec3(v: deepspace::Vec3) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

/// 将 macroquad Vec3 (f32) 转换为 deepspace::Vec3 (f64)
#[allow(dead_code)]
pub fn from_mvec3(v: Vec3) -> deepspace::Vec3 {
    deepspace::Vec3::new(v.x as f64, v.y as f64, v.z as f64)
}

// =====================================================================
// 统一 HUD — 多类型物理模拟互通显示层
// =====================================================================

/// 实体类型对应的 HUD 颜色
pub fn entity_kind_color(kind: deepspace::entity::EntityKind) -> Color {
    use deepspace::entity::EntityKind::*;
    match kind {
        Rocket => Color::new(0.4, 0.9, 1.0, 1.0),     // 青 — 火箭
        Spacecraft => Color::new(0.6, 0.8, 1.0, 1.0), // 亮蓝 — 航天器
        Missile => Color::new(1.0, 0.4, 0.3, 1.0),    // 红 — 拦截导弹
        Icbm => Color::new(1.0, 0.7, 0.2, 1.0),       // 橙 — 弹道目标
        Aircraft => Color::new(0.4, 1.0, 0.5, 1.0),   // 绿 — 飞机
        Body => Color::new(0.8, 0.8, 0.8, 1.0),       // 灰 — 天体
    }
}

/// 单行实体遥测（统一视图，任何类型都可显示）
pub struct EntityHudRow {
    pub kind: deepspace::entity::EntityKind,
    pub name: String,
    pub altitude_km: f64,
    pub speed_mps: f64,
    pub status: String,
    pub alive: bool,
}

impl EntityHudRow {
    pub fn from_entity(e: &deepspace::entity::Entity) -> Self {
        Self {
            kind: e.kind,
            name: e.name.clone(),
            altitude_km: e.altitude_m / 1000.0,
            speed_mps: e.speed_mps,
            status: e.status.clone(),
            alive: e.alive,
        }
    }
}

/// 绘制统一实体遥测面板（左上）— 所有类型共享一个表格
pub fn draw_entity_hud_panel(rows: &[EntityHudRow], x: f32, y: f32) {
    let s = ui_scale();
    let row_h = 26.0 * s;
    let col_x = [
        x + 12.0 * s,  // 类型色块 + 名称
        x + 170.0 * s, // 高度
        x + 260.0 * s, // 速度
        x + 350.0 * s, // 状态
    ];
    let panel_w = 520.0 * s;
    let content_h = row_h * rows.len() as f32 + 34.0 * s;

    draw_rectangle(
        x - 8.0 * s,
        y - 28.0 * s,
        panel_w + 16.0 * s,
        content_h + 8.0 * s,
        Color::new(0.05, 0.06, 0.09, 0.85),
    );
    text(
        "实体遥测 (UNIFIED)",
        x,
        y - 8.0 * s,
        15.0 * s,
        Color::new(0.7, 0.8, 1.0, 1.0),
    );

    let mut cy = y + 12.0 * s;
    for row in rows {
        let c = entity_kind_color(row.kind);
        let alive_c = if row.alive {
            c
        } else {
            Color::new(0.4, 0.4, 0.4, 1.0)
        };
        // 类型色块
        draw_rectangle(x, cy - 14.0 * s, 10.0 * s, 10.0 * s, alive_c);
        // 名称
        text(&row.name, col_x[0] + 16.0 * s, cy, 14.0 * s, alive_c);
        // 高度
        text(
            format!("{:>7.1} km", row.altitude_km),
            col_x[1],
            cy,
            14.0 * s,
            Color::new(0.9, 0.9, 0.9, 1.0),
        );
        // 速度
        text(
            format!("{:>6.0} m/s", row.speed_mps),
            col_x[2],
            cy,
            14.0 * s,
            Color::new(0.9, 0.9, 0.9, 1.0),
        );
        // 状态
        text(
            &row.status,
            col_x[3],
            cy,
            13.0 * s,
            Color::new(0.65, 0.75, 0.85, 1.0),
        );
        cy += row_h;
    }
}

/// 绘制世界事件日志面板（左下）— 闭环反馈流
pub fn draw_event_log_panel(events: &[String], x: f32, y: f32) {
    let s = ui_scale();
    let row_h = 20.0 * s;
    let max_rows = 8;
    let show = events.iter().rev().take(max_rows);
    let content_h = row_h * (show.clone().count() as f32) + 30.0 * s;
    let panel_w = 460.0 * s;

    draw_rectangle(
        x - 8.0 * s,
        y - 26.0 * s,
        panel_w + 16.0 * s,
        content_h + 8.0 * s,
        Color::new(0.05, 0.06, 0.09, 0.85),
    );
    text(
        "事件日志 (EVENTS)",
        x,
        y - 6.0 * s,
        15.0 * s,
        Color::new(1.0, 0.85, 0.5, 1.0),
    );

    let mut cy = y + 10.0 * s;
    for ev in show {
        text(
            ev,
            x + 4.0 * s,
            cy,
            13.0 * s,
            Color::new(0.8, 0.85, 0.9, 1.0),
        );
        cy += row_h;
    }
}

/// 绘制世界状态条（顶部）— 时间 / 实体数 / 事件数
pub fn draw_world_status_bar(time_s: f64, entity_count: usize, event_count: usize) {
    let s = ui_scale();
    let label = format!(
        "T+{:>7.1}s   实体 {:>2}   事件 {:>3}",
        time_s, entity_count, event_count
    );
    draw_rectangle(
        0.0,
        0.0,
        screen_width(),
        30.0 * s,
        Color::new(0.05, 0.06, 0.09, 0.9),
    );
    text(
        &label,
        12.0 * s,
        20.0 * s,
        16.0 * s,
        Color::new(0.75, 0.9, 1.0, 1.0),
    );
}

// =====================================================================
// NASA Eyes 风格渲染原语
//
// 参考视觉：https://eyes.nasa.gov/apps/asteroids/ 与
//          https://eyes.nasa.gov/apps/solar-system/
// 深空黑背景 + 恒星视差 + 发光天体 + 半透明轨道环 + 渐变轨迹
// + 底部时间控制条 + 跟随选中光环
// =====================================================================

/// 简单 xorshift RNG（不引入外部依赖）
struct Xorshift(u64);

impl Xorshift {
    fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
    /// [0,1) 均匀
    fn f32(&mut self) -> f32 {
        (self.next() >> 40) as f32 / (1u64 << 24) as f32
    }
}

/// NASA Eyes 深空背景色（--bgGradient 底部 #070709 近纯黑暖调）
pub const COLOR_SPACE_BG: Color = Color::new(0.027, 0.027, 0.035, 1.0);

/// 静态星空 — 单位球面上的固定星点，随相机旋转产生视差
pub struct StarField {
    stars: Vec<(f32, f32, f32, f32)>, // dir_x, dir_y, dir_z, brightness
}

impl StarField {
    pub fn new(count: usize, seed: u64) -> Self {
        let mut rng = Xorshift::new(seed);
        let mut stars = Vec::with_capacity(count);
        for _ in 0..count {
            // 均匀球面分布（z + 方位角）
            let z = rng.f32() * 2.0 - 1.0;
            let a = rng.f32() * std::f32::consts::TAU;
            let r = (1.0 - z * z).sqrt();
            let b = 0.25 + rng.f32() * 0.75; // 亮度 0.25~1.0
            stars.push((r * a.cos(), z, r * a.sin(), b));
        }
        Self { stars }
    }

    /// 绘制星空。星点放在以相机目标为中心的远距离球面上，
    /// 距离随相机 distance 缩放，保证永远在场景之后且投影稳定。
    pub fn draw(&self, camera: &OrbitalCamera, sw: f32, sh: f32) {
        let far = (camera.distance * 800.0).max(1.0e7);
        let base = ui_scale();
        for &(dx, dy, dz, b) in &self.stars {
            let pos = camera.target + Vec3::new(dx, dy, dz) * far;
            let (x, y) = camera.project_2d(pos, sw, sh);
            if x < -4.0 || x > sw + 4.0 || y < -4.0 || y > sh + 4.0 {
                continue;
            }
            // 亮度越高星点越大越亮，带轻微冷色调
            let size = if b > 0.85 {
                1.8 * base
            } else if b > 0.6 {
                1.3 * base
            } else {
                0.9 * base
            };
            draw_rectangle(
                x,
                y,
                size,
                size,
                Color::new(b * 0.85, b * 0.88, b * 1.0, b * 0.9),
            );
        }
    }
}

/// 绘制发光天体 — NASA Eyes 风格的光晕分层（白热核心 + 色晕）
///
/// - `cx`, `cy`: 屏幕中心
/// - `r_px`: 天体核心半径（像素）
/// - `color`: 天体本色（光晕用同色低 alpha 扩散）
/// - `intensity`: 光晕强度（太阳等大热源 >1，行星 ~1，小物体 <1）
pub fn draw_glow_2d(cx: f32, cy: f32, r_px: f32, color: Color, intensity: f32) {
    if r_px < 0.5 {
        return;
    }
    // 外光晕（3 层衰减）
    draw_circle_2d(
        cx,
        cy,
        r_px * 3.4,
        Color::new(color.r, color.g, color.b, 0.08 * intensity),
    );
    draw_circle_2d(
        cx,
        cy,
        r_px * 2.1,
        Color::new(color.r, color.g, color.b, 0.18 * intensity),
    );
    draw_circle_2d(
        cx,
        cy,
        r_px * 1.3,
        Color::new(color.r, color.g, color.b, 0.4 * intensity),
    );
    // 白热核心（NASA Eyes 天体中心偏白）
    draw_circle_2d(cx, cy, r_px * 0.7, Color::new(0.95, 0.97, 1.0, 0.85));
    // 本体
    draw_circle_2d(cx, cy, r_px, color);
}

/// 绘制轨道环 — 3D 圆在透视下的椭圆投影（细线、半透明）
///
/// 对应 asteroids 首页每颗小行星一条轨道环的视觉。
/// 轨道平面由 `normal` 决定（默认 Y 轴 = XZ 平面）。
pub fn draw_orbit_ring_2d(
    camera: &OrbitalCamera,
    center: Vec3,
    radius: f32,
    normal: Vec3,
    sw: f32,
    sh: f32,
    color: Color,
) {
    if radius <= 0.0 {
        return;
    }
    let n = normal.normalize();
    // 构造轨道平面正交基
    let u = if n.y.abs() > 0.95 {
        n.cross(Vec3::X).normalize()
    } else {
        n.cross(Vec3::Y).normalize()
    };
    let v = n.cross(u).normalize();

    let segs = 96u32;
    let step = std::f32::consts::TAU / segs as f32;
    let mut prev = None;
    for i in 0..=segs {
        let a = (i % segs) as f32 * step;
        let p3d = center + (u * a.cos() + v * a.sin()) * radius;
        let (x, y) = camera.project_2d(p3d, sw, sh);
        if let Some((px, py)) = prev {
            draw_line(px, py, x, y, 1.0, color);
        }
        prev = Some((x, y));
    }
}

/// 线性插值颜色
fn lerp_color(a: Color, b: Color, t: f32) -> Color {
    let t = t.clamp(0.0, 1.0);
    Color::new(
        a.r + (b.r - a.r) * t,
        a.g + (b.g - a.g) * t,
        a.b + (b.b - a.b) * t,
        a.a + (b.a - a.a) * t,
    )
}

/// 绘制渐变轨迹线 — 老点偏蓝、新点偏橙（NASA Eyes 速度梯度）
///
/// 每段按点在序列中的位置插值 `old_color` → `new_color`。
pub fn draw_gradient_path_2d(
    camera: &OrbitalCamera,
    points: &[Vec3],
    sw: f32,
    sh: f32,
    old_color: Color,
    new_color: Color,
) {
    let n = points.len();
    if n < 2 {
        return;
    }
    let denom = (n - 1) as f32;
    for (i, w) in points.windows(2).enumerate() {
        let t = i as f32 / denom;
        let c = lerp_color(old_color, new_color, t);
        let (x1, y1) = camera.project_2d(w[0], sw, sh);
        let (x2, y2) = camera.project_2d(w[1], sw, sh);
        draw_line(x1, y1, x2, y2, 1.2, c);
    }
}

/// 绘制选中光环 — NASA Eyes 跟随目标的白色双环准星
pub fn draw_selection_ring(cx: f32, cy: f32, r_px: f32) {
    if r_px < 2.0 {
        return;
    }
    let c = Color::new(0.85, 0.9, 1.0, 0.85);
    draw_circle_2d(cx, cy, r_px * 1.9, c);
    draw_circle_2d(cx, cy, r_px * 0.9, Color::new(0.85, 0.9, 1.0, 0.35));
    // 十字准星
    let cross = (r_px * 2.6).max(6.0);
    draw_line(cx - cross, cy, cx + cross, cy, 1.0, c);
    draw_line(cx, cy - cross, cx, cy + cross, 1.0, c);
}

/// 把秒格式化为 "T+ 1d 05:12:33"（NASA Eyes 风格 UTC 读数）
pub fn format_sim_time(time_s: f64) -> String {
    let t = time_s.abs().max(0.0) as u64;
    let days = t / 86_400;
    let h = (t % 86_400) / 3_600;
    let m = (t % 3_600) / 60;
    let sec = t % 60;
    if days > 0 {
        format!("T+ {}d {:02}:{:02}:{:02}", days, h, m, sec)
    } else {
        format!("T+ {:02}:{:02}:{:02}", h, m, sec)
    }
}

/// NASA Eyes 底部时间控制条 — 播放/暂停 + 时间倍率 + 模拟时间
///
/// 按键：Space 播放/暂停，`+`/`=` 倍率 ×2，`-` 倍率 ÷2。
#[derive(Clone)]
pub struct TimeControlBar {
    pub paused: bool,
    pub rate: f64,
}

impl Default for TimeControlBar {
    fn default() -> Self {
        Self::new()
    }
}

impl TimeControlBar {
    pub fn new() -> Self {
        Self {
            paused: false,
            rate: 1.0,
        }
    }
    /// 读取键盘更新状态，返回是否发生了暂停切换（用于音效/闪烁）
    pub fn update(&mut self) -> bool {
        let mut toggled = false;
        if is_key_pressed(KeyCode::Space) {
            self.paused = !self.paused;
            toggled = true;
        }
        if is_key_pressed(KeyCode::Equal) || is_key_pressed(KeyCode::KpAdd) {
            self.rate = (self.rate * 2.0).min(1.0e6);
        }
        if is_key_pressed(KeyCode::Minus) || is_key_pressed(KeyCode::KpSubtract) {
            self.rate = (self.rate / 2.0).max(0.001);
        }
        toggled
    }

    /// 绘制底部细条（右侧叠加速率、左侧模拟时间）
    pub fn draw(&self, time_s: f64) {
        let s = ui_scale();
        let w = screen_width();
        let h = 34.0 * s;
        let y = screen_height() - h;

        // 半透明深色底 + 顶边高光细线（NASA Eyes --grayDark + divider）
        draw_rectangle(0.0, y, w, h, COLOR_PANEL_BG);
        draw_line(0.0, y, w, y, 1.0, Color::new(0.216, 0.216, 0.227, 0.7));

        // 播放/暂停图标
        let ic = Color::new(0.75, 0.9, 1.0, 0.95);
        let (icon, ix) = if self.paused {
            ("▶", 22.0 * s)
        } else {
            ("⏸", 18.0 * s)
        };
        text(icon, ix, y + h * 0.62, 20.0 * s, ic);

        // 模拟时间（左侧，居中于条）
        let tlabel = format_sim_time(time_s);
        let tw = 240.0 * s;
        text(
            &tlabel,
            w / 2.0 - tw / 2.0,
            y + h * 0.62,
            18.0 * s,
            Color::new(0.9, 0.94, 1.0, 1.0),
        );

        // 倍率（右侧）
        let rate_label = if self.rate.abs() < 1.0 {
            format!("×{:.3}", self.rate)
        } else {
            format!("×{:.0}", self.rate)
        };
        let rate_color = if self.paused {
            Color::new(0.6, 0.65, 0.75, 0.9)
        } else {
            Color::new(1.0, 0.85, 0.4, 1.0)
        };
        let rate_w = 120.0 * s;
        text(
            &rate_label,
            w - rate_w - 16.0 * s,
            y + h * 0.62,
            18.0 * s,
            rate_color,
        );

        // 快捷键提示（右下角，弱化）
        text(
            "Space 暂停  +/− 倍率  ESC 退出",
            w - 320.0 * s,
            y + h * 0.28,
            12.0 * s,
            Color::new(0.4, 0.5, 0.6, 0.8),
        );
    }
}

/// 左键短按检测 — 区分"拖拽旋转"与"点击选中"
///
/// 返回 true 表示本次点击（按下到释放位移 < 5px）。
#[derive(Clone, Default)]
pub struct ClickDetector {
    pressed_pos: Option<(f32, f32)>,
}

impl ClickDetector {
    pub fn new() -> Self {
        Self { pressed_pos: None }
    }

    /// 每帧调用；返回 true 表示一次有效的点击
    pub fn update(&mut self) -> bool {
        if is_mouse_button_pressed(MouseButton::Left) {
            self.pressed_pos = Some(mouse_position());
        }
        if is_mouse_button_released(MouseButton::Left) {
            if let Some((px, py)) = self.pressed_pos.take() {
                let (mx, my) = mouse_position();
                let dx = mx - px;
                let dy = my - py;
                if dx * dx + dy * dy < 25.0 {
                    return true;
                }
            }
        }
        false
    }
}

/// 深色半透明面板底色（NASA Eyes HUD --grayDark:#252527 暖深灰）
pub const COLOR_PANEL_BG: Color = Color::new(0.145, 0.145, 0.153, 0.92);

/// 绘制面板背景 + 细边框（NASA Eyes HUD 风格，--grayDivider:#37373a）
pub fn draw_panel(x: f32, y: f32, w: f32, h: f32) {
    draw_rectangle(x, y, w, h, COLOR_PANEL_BG);
    // 边框（上 + 左高光、下 + 右暗边）
    let border = Color::new(0.216, 0.216, 0.227, 0.9); // #37373a
    draw_line(x, y, x + w, y, 1.0, border);
    draw_line(x, y, x, y + h, 1.0, border);
    draw_line(
        x,
        y + h,
        x + w,
        y + h,
        1.0,
        Color::new(0.216, 0.216, 0.227, 0.55),
    );
    draw_line(
        x + w,
        y,
        x + w,
        y + h,
        1.0,
        Color::new(0.216, 0.216, 0.227, 0.55),
    );
}
