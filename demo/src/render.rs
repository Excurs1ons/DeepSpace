//! 3D 渲染辅助 — 在 macroquad 之上封装太空可视化原语
//!
//! 提供轨道相机、行星绘制、姿态 Gizmo、轨迹线等功能。

use macroquad::color::Color;
use macroquad::math::{Quat, Vec3};
use macroquad::models::{
    draw_line_3d, draw_sphere,
};
use macroquad::shapes::draw_line;
use macroquad::text::draw_text;
use macroquad::camera::{set_camera, set_default_camera, Camera3D};
use macroquad::input::{
    is_mouse_button_down, mouse_position, mouse_wheel, MouseButton,
};

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
                self.elevation = (self.elevation + dy * self.sensitivity)
                    .clamp(-1.5, 1.5);
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
            self.target_distance = self.target_distance.clamp(self.min_distance, self.max_distance);
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

pub const COLOR_SUN: Color = Color::new(1.0, 0.8, 0.0, 1.0);
pub const COLOR_MERCURY: Color = Color::new(0.7, 0.7, 0.7, 1.0);
pub const COLOR_VENUS: Color = Color::new(0.9, 0.7, 0.3, 1.0);
pub const COLOR_EARTH: Color = Color::new(0.2, 0.4, 0.8, 1.0);
pub const COLOR_MARS: Color = Color::new(0.8, 0.3, 0.2, 1.0);
pub const COLOR_JUPITER: Color = Color::new(0.8, 0.6, 0.4, 1.0);
pub const COLOR_MOON: Color = Color::new(0.6, 0.6, 0.6, 1.0);
pub const COLOR_SHIP: Color = Color::new(0.9, 0.9, 0.9, 1.0);
pub const COLOR_PATH: Color = Color::new(0.3, 0.8, 1.0, 0.6);
pub const COLOR_GIZMO_X: Color = Color::new(1.0, 0.2, 0.2, 1.0);
pub const COLOR_GIZMO_Y: Color = Color::new(0.2, 1.0, 0.2, 1.0);
pub const COLOR_GIZMO_Z: Color = Color::new(0.2, 0.2, 1.0, 1.0);
pub const COLOR_TRAJECTORY: Color = Color::new(0.3, 0.8, 1.0, 0.8);
pub const COLOR_PREDICTION: Color = Color::new(0.3, 0.8, 1.0, 0.25);
pub const COLOR_GRID: Color = Color::new(0.15, 0.18, 0.25, 0.5);
pub const COLOR_GRID_AXIS: Color = Color::new(0.25, 0.3, 0.4, 0.6);
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
/// - `duration`: 预测时长 (s)
/// - `num_points`: 采样点数（决定线条平滑度）
/// - `earth_radius`: 若 > 0，轨迹进入地表以下时截断（避免穿透地球）
/// - 返回惯性系中的位置序列
pub fn predict_trajectory(
    pos: deepspace::Vec3,
    vel: deepspace::Vec3,
    mu: f64,
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
        s = rk4_twobody(&s, mu, dt);
        let p = deepspace::Vec3::new(s[0], s[1], s[2]);
        // 截断：进入地表以下则停止
        if earth_radius > 0.0 && p.length() < earth_radius {
            break;
        }
        points.push(p);
    }
    points
}

/// RK4 单步（二体引力）
fn rk4_twobody(s: &[f64; 6], mu: f64, dt: f64) -> [f64; 6] {
    let deriv = |y: &[f64; 6]| -> [f64; 6] {
        let r2 = y[0] * y[0] + y[1] * y[1] + y[2] * y[2];
        if r2 > 0.0 {
            let a = -mu / (r2 * r2.sqrt());
            [y[3], y[4], y[5], y[0] * a, y[1] * a, y[2] * a]
        } else {
            [y[3], y[4], y[5], 0.0, 0.0, 0.0]
        }
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
        let p1 = Vec3::new(points[i + 1].x as f32, points[i + 1].y as f32, points[i + 1].z as f32);
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
        draw_text(label, ex - 5.0, ey - 5.0, 11.0, c);
    }
}

// =====================================================================
// 2D 正交投影渲染 — 整个场景为 2D HUD
// =====================================================================

/// 画 2D 圆环（线段近似）
pub fn draw_circle_2d(cx: f32, cy: f32, radius: f32, segments: u32, color: Color) {
    if radius < 0.5 {
        draw_line(cx, cy, cx, cy, 1.0, color);
        return;
    }
    let step = std::f32::consts::TAU / segments.max(6) as f32;
    for i in 0..segments {
        let a0 = i as f32 * step;
        let a1 = (i + 1) as f32 * step;
        let (s0, c0) = a0.sin_cos();
        let (s1, c1) = a1.sin_cos();
        draw_line(
            cx + c0 * radius, cy + s0 * radius,
            cx + c1 * radius, cy + s1 * radius,
            1.0, color,
        );
    }
}

/// 投影绘制地球（2D 圆 + 中心十字）
pub fn draw_earth_2d(camera: &OrbitalCamera, earth_radius: f32, sw: f32, sh: f32) {
    let (cx, cy) = camera.project_2d(Vec3::ZERO, sw, sh);
    let r = camera.len_to_px(earth_radius, sw, sh);
    // 地球轮廓
    draw_circle_2d(cx, cy, r, 48, COLOR_EARTH);
    // 地心十字
    let cross = 6.0_f32.max(r * 0.04);
    draw_line(cx - cross, cy, cx + cross, cy, 1.0, COLOR_EARTH);
    draw_line(cx, cy - cross, cx, cy + cross, 1.0, COLOR_EARTH);
}

/// 投影绘制空间参考网格
///
/// 将 3D 赤道面同心环 + 辐条 + 坐标轴投影到 2D 屏幕。
/// 旋转相机时呈现不同剖面（椭圆/线）。
pub fn draw_grid_2d(camera: &OrbitalCamera, earth_radius: f32, sw: f32, sh: f32) {
    const RINGS: [f32; 5] = [1.5, 2.0, 3.0, 5.0, 10.0];
    const SEGMENTS: u32 = 48;
    const SPOKES: u32 = 12;
    const CENTER: Vec3 = Vec3::ZERO;

    // 1. 同心环投影到 2D
    for &mult in &RINGS {
        let r = earth_radius * mult;
        let step = std::f32::consts::TAU / SEGMENTS as f32;
        let mut prev = None;
        for i in 0..=SEGMENTS {
            let a = ((i % SEGMENTS) as f32) * step;
            let p3d = CENTER + Vec3::new(a.cos() * r, 0.0, a.sin() * r);
            let (x, y) = camera.project_2d(p3d, sw, sh);
            if let Some((px, py)) = prev {
                draw_line(px, py, x, y, 1.0, COLOR_GRID);
            }
            prev = Some((x, y));
        }
    }

    // 2. 径向辐条
    let max_r = earth_radius * 10.0;
    let (cx, cy) = camera.project_2d(CENTER, sw, sh);
    for i in 0..SPOKES {
        let a = (i as f32 / SPOKES as f32) * std::f32::consts::TAU;
        let end = CENTER + Vec3::new(a.cos() * max_r, 0.0, a.sin() * max_r);
        let (ex, ey) = camera.project_2d(end, sw, sh);
        draw_line(cx, cy, ex, ey, 1.0, COLOR_GRID);
    }

    // 3. 坐标轴线
    let a_len = earth_radius * 12.0;
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
    draw_text("X", xp.0 + 3.0, xp.1 - 3.0, 10.0, COLOR_GRID_AXIS);
    draw_text("Y", yp.0 + 3.0, yp.1 - 3.0, 10.0, COLOR_GRID_AXIS);
    draw_text("Z", zp.0 + 3.0, zp.1 - 3.0, 10.0, COLOR_GRID_AXIS);
}

/// 投影绘制 2D 轨迹线
pub fn draw_path_2d(
    camera: &OrbitalCamera,
    points: &[Vec3],
    sw: f32, sh: f32,
    color: Color,
) {
    for w in points.windows(2) {
        let (x1, y1) = camera.project_2d(w[0], sw, sh);
        let (x2, y2) = camera.project_2d(w[1], sw, sh);
        draw_line(x1, y1, x2, y2, 1.0, color);
    }
}

/// 投影绘制预测轨道（虚线隔段画）
pub fn draw_predicted_path_2d(
    camera: &OrbitalCamera,
    points: &[deepspace::Vec3],
    sw: f32, sh: f32,
    color: Color,
) {
    if points.len() < 2 {
        return;
    }
    for i in (0..points.len() - 1).step_by(2) {
        let p0 = Vec3::new(points[i].x as f32, points[i].y as f32, points[i].z as f32);
        let p1 = Vec3::new(points[i + 1].x as f32, points[i + 1].y as f32, points[i + 1].z as f32);
        let (x1, y1) = camera.project_2d(p0, sw, sh);
        let (x2, y2) = camera.project_2d(p1, sw, sh);
        draw_line(x1, y1, x2, y2, 1.0, color);
    }
}

/// 投影绘制火箭标记 + 速度方向箭头
pub fn draw_rocket_2d(
    camera: &OrbitalCamera,
    pos: Vec3,
    vel: Vec3,
    earth_radius: f32,
    sw: f32, sh: f32,
) {
    let (rx, ry) = camera.project_2d(pos, sw, sh);

    // 火箭位置圆点
    let marker_r = camera.len_to_px(2000.0_f32.max(earth_radius * 0.003), sw, sh).max(2.5);
    draw_circle_2d(rx, ry, marker_r, 10, COLOR_SHIP);

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
            ax, ay,
            ax - nx * head + ny * head * 0.5,
            ay + ny * head + nx * head * 0.5,
            1.5, COLOR_GIZMO_Y,
        );
        draw_line(
            ax, ay,
            ax - nx * head - ny * head * 0.5,
            ay + ny * head - nx * head * 0.5,
            1.5, COLOR_GIZMO_Y,
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
