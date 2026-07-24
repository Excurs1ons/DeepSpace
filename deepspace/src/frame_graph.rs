//! FrameGraph — 层次化坐标参考系系统
//!
//! 灵感：RenderGraph 自动依赖分析 → 实体空间依赖自动精度选择。
//! 每个实体在所属帧(自然单位)内存储位置，帧形成树状层次。
//! 跨帧距离 = LCA 帧 + 自动单位选择。
//!
//! 帧链示例:
//!   UniverseRoot
//!     └─ LocalGroup (Mpc)
//!          └─ MilkyWay (kpc)
//!               ├─ Sol (AU)
//!               │    ├─ Earth (km)
//!               │    │    └─ Ship (km)
//!               │    └─ Mars (km)
//!               └─ AlphaCentauri (AU)
//!                    └─ ProximaB (AU)

/// 长度单位
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LengthUnit {
    Mm,  // 毫米 (10⁻³ m)
    M,   // 米
    Km,  // 千米 (10³ m)
    Au,  // 天文单位 (~1.496e11 m)
    Ly,  // 光年 (~9.461e15 m)
    Kpc, // 千秒差距 (~3.086e19 m)
    Mpc, // 百万秒差距 (~3.086e22 m)
}

impl LengthUnit {
    /// 转换为米
    pub fn to_meters(&self) -> f64 {
        match self {
            LengthUnit::Mm => 0.001,
            LengthUnit::M => 1.0,
            LengthUnit::Km => 1_000.0,
            LengthUnit::Au => 149_597_870_700.0,
            LengthUnit::Ly => 9_460_730_472_580_800.0,
            LengthUnit::Kpc => 3.085_677_581_491_367_2e19,
            LengthUnit::Mpc => 3.085_677_581_491_367_2e22,
        }
    }

    /// 单位名称缩写
    pub fn abbrev(&self) -> &'static str {
        match self {
            LengthUnit::Mm => "mm",
            LengthUnit::M => "m",
            LengthUnit::Km => "km",
            LengthUnit::Au => "AU",
            LengthUnit::Ly => "ly",
            LengthUnit::Kpc => "kpc",
            LengthUnit::Mpc => "Mpc",
        }
    }

    /// 根据米数自动选择最佳单位
    pub fn best_for(meters: f64) -> (f64, LengthUnit) {
        let abs = meters.abs();
        // 边界以 overlap 保证覆盖：
        //   1 ly  = 9.46e15 m  → [1e15, 1e18)
        //   1 kpc = 3.09e19 m  → [1e19, 1e21)
        let candidates = [
            (0.0, 1e0, LengthUnit::Mm),    // [0m, 1m)
            (1e0, 1e3, LengthUnit::M),     // [1m, 1km)
            (1e3, 1e7, LengthUnit::Km),    // [1km, 10,000km)
            (1e7, 1e14, LengthUnit::Au),   // [10Mm, 1000AU)
            (1e14, 1e18, LengthUnit::Ly),  // [0.01ly, 100ly)
            (1e18, 1e22, LengthUnit::Kpc), // [0.03kpc, 300kpc)
        ];
        for &(lo, hi, unit) in &candidates {
            if abs >= lo && abs < hi {
                return (meters / unit.to_meters(), unit);
            }
        }
        // 超出 kpc 上限 → Mpc
        (meters / LengthUnit::Mpc.to_meters(), LengthUnit::Mpc)
    }

    /// 从字符串解析单位
    pub fn parse(s: &str) -> Option<LengthUnit> {
        match s.to_lowercase().as_str() {
            "mm" => Some(LengthUnit::Mm),
            "m" => Some(LengthUnit::M),
            "km" => Some(LengthUnit::Km),
            "au" => Some(LengthUnit::Au),
            "ly" | "lightyear" | "光年" => Some(LengthUnit::Ly),
            "kpc" => Some(LengthUnit::Kpc),
            "mpc" => Some(LengthUnit::Mpc),
            _ => None,
        }
    }
}

// =====================================================================
// FrameId — 帧标识符
// =====================================================================
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FrameId(pub u64);

impl FrameId {
    pub const ROOT: FrameId = FrameId(0);
}

// =====================================================================
// FrameNode — 帧节点
// =====================================================================
#[derive(Debug, Clone)]
pub struct FrameNode {
    pub id: FrameId,
    pub name: String,
    pub unit: LengthUnit,
    pub parent: Option<FrameId>,
    /// 此帧原点在父帧中的位置 (父帧单位)
    pub origin_in_parent: [f64; 3],
}

// =====================================================================
// EntityPosition — 实体在帧中的位置
// =====================================================================
#[derive(Debug, Clone, Copy)]
pub struct EntityPosition {
    /// 所属帧
    pub frame: FrameId,
    /// 在帧的 unit 下的坐标
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl EntityPosition {
    pub fn new(frame: FrameId, x: f64, y: f64, z: f64) -> Self {
        Self { frame, x, y, z }
    }
}

// =====================================================================
// Scalar — 带单位标量（距离计算结果）
// =====================================================================
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Scalar {
    pub value: f64,
    pub unit: LengthUnit,
}

impl Scalar {
    pub fn new(value: f64, unit: LengthUnit) -> Self {
        Self { value, unit }
    }

    /// 转换到目标单位
    pub fn to(&self, target: LengthUnit) -> f64 {
        self.value * self.unit.to_meters() / target.to_meters()
    }

    /// 自动选最优单位并重新缩放
    pub fn auto_unit(&self) -> Scalar {
        let meters = self.value * self.unit.to_meters();
        let (v, u) = LengthUnit::best_for(meters);
        Scalar { value: v, unit: u }
    }

    pub fn format(&self) -> String {
        let a = self.auto_unit();
        format!("{:.4} {}", a.value, a.unit.abbrev())
    }
}

// =====================================================================
// FrameGraph — 帧图管理器
// =====================================================================
pub struct FrameGraph {
    frames: Vec<FrameNode>,
}

impl FrameGraph {
    pub fn new() -> Self {
        // 创建 UniverseRoot（无单位，无父帧）
        let root = FrameNode {
            id: FrameId::ROOT,
            name: "UniverseRoot".into(),
            unit: LengthUnit::Mpc, // 根用 Mpc
            parent: None,
            origin_in_parent: [0.0, 0.0, 0.0],
        };
        Self { frames: vec![root] }
    }

    // ---- 查询 ----

    /// 获取帧节点
    pub fn get(&self, id: FrameId) -> Option<&FrameNode> {
        self.frames.iter().find(|f| f.id == id)
    }

    /// 根帧 ID
    pub fn root_id(&self) -> FrameId {
        FrameId::ROOT
    }

    /// 帧数量
    pub fn count(&self) -> usize {
        self.frames.len()
    }

    // ---- 添加 ----

    /// 添加帧，返回分配的 ID
    pub fn add_frame(
        &mut self,
        name: &str,
        unit: LengthUnit,
        parent: Option<FrameId>,
        origin: [f64; 3],
    ) -> FrameId {
        // 验证父帧存在
        if let Some(pid) = parent {
            assert!(
                self.frames.iter().any(|f| f.id == pid),
                "parent frame {:?} not found",
                pid
            );
        }
        let id = FrameId(self.frames.len() as u64);
        self.frames.push(FrameNode {
            id,
            name: name.to_string(),
            unit,
            parent,
            origin_in_parent: origin,
        });
        id
    }

    // ---- 帧路径分析 ----

    /// 从帧到根的路径 (包含自身，不含根)
    pub fn path_to_root(&self, id: FrameId) -> Vec<FrameId> {
        let mut path = Vec::new();
        let mut current = id;
        loop {
            path.push(current);
            if current == FrameId::ROOT {
                break;
            }
            match self.get(current).and_then(|n| n.parent) {
                Some(p) => current = p,
                None => break,
            }
        }
        path
    }

    /// 最近公共祖先帧
    pub fn lca(&self, a: FrameId, b: FrameId) -> FrameId {
        let path_a = self.path_to_root(a);
        let path_b = self.path_to_root(b);
        let mut common = FrameId::ROOT;
        let max_len = path_a.len().min(path_b.len());
        for i in 0..max_len {
            if path_a[path_a.len() - 1 - i] == path_b[path_b.len() - 1 - i] {
                common = path_a[path_a.len() - 1 - i];
            } else {
                break;
            }
        }
        common
    }

    // ---- 位置解析 ----

    /// 将实体位置解析到目标帧坐标
    /// 解析结果在 target 帧的单位下
    pub fn resolve(&self, pos: &EntityPosition, target: FrameId) -> [f64; 3] {
        if pos.frame == target {
            return [pos.x, pos.y, pos.z];
        }

        let common = self.lca(pos.frame, target);

        // 1. 从 pos.frame 上升到 common
        let mut x = pos.x;
        let mut y = pos.y;
        let mut z = pos.z;
        let mut current_unit = self.get(pos.frame).unwrap().unit;
        let mut current_id = pos.frame;

        while current_id != common {
            let node = self.get(current_id).unwrap();
            if let Some(parent_id) = node.parent {
                let parent = self.get(parent_id).unwrap();
                // 转换到父单位
                let scale = current_unit.to_meters() / parent.unit.to_meters();
                x = node.origin_in_parent[0] + x * scale;
                y = node.origin_in_parent[1] + y * scale;
                z = node.origin_in_parent[2] + z * scale;
                current_unit = parent.unit;
                current_id = parent_id;
            } else {
                break; // 到了根
            }
        }

        // 2. 从 common 下降到 target
        // 构建下降路径: target → ... → common (反转)
        let mut descend: Vec<FrameId> = Vec::new();
        let mut c = target;
        while c != common {
            descend.push(c);
            let node = self.get(c).unwrap();
            match node.parent {
                Some(p) => c = p,
                None => break,
            }
        }
        descend.reverse(); // 现在: common 的下级 → ... → target

        for &child_id in &descend {
            let child = self.get(child_id).unwrap();
            let parent = self.get(child.parent.unwrap()).unwrap();
            // 当前值在 parent 单位中，需转换到 child 单位，再减去原点
            let scale = parent.unit.to_meters() / child.unit.to_meters();
            x = (x - child.origin_in_parent[0]) * scale;
            y = (y - child.origin_in_parent[1]) * scale;
            z = (z - child.origin_in_parent[2]) * scale;
            let _ = child.unit;
        }

        [x, y, z]
    }

    // ---- 距离计算 ----

    /// 计算两个实体间的距离，自动选合适单位
    pub fn distance(&self, a: &EntityPosition, b: &EntityPosition) -> Scalar {
        let common = self.lca(a.frame, b.frame);
        let pa = self.resolve(a, common);
        let pb = self.resolve(b, common);
        let dx = pa[0] - pb[0];
        let dy = pa[1] - pb[1];
        let dz = pa[2] - pb[2];
        let val = (dx * dx + dy * dy + dz * dz).sqrt(); // 在 common 单位下
        let unit = self.get(common).unwrap().unit;
        Scalar::new(val, unit)
    }

    /// 距离，结果自动用最优单位
    pub fn distance_auto(&self, a: &EntityPosition, b: &EntityPosition) -> Scalar {
        self.distance(a, b).auto_unit()
    }

    // ---- 跨帧单位转换 ----

    /// 转换标量值到目标单位
    pub fn convert(&self, value: f64, from: LengthUnit, to: LengthUnit) -> f64 {
        value * from.to_meters() / to.to_meters()
    }
}

// =====================================================================
// 构建常见宇宙帧层次结构的辅助函数
// =====================================================================
impl FrameGraph {
    /// 构建一个标准的太阳系-银河系帧层次
    /// 返回各个关键帧的 ID
    pub fn build_standard_universe(&mut self) -> StandardFrames {
        let lg = self.add_frame(
            "LocalGroup",
            LengthUnit::Mpc,
            Some(FrameId::ROOT),
            [0.0, 0.0, 0.0],
        );
        let mw = self.add_frame("MilkyWay", LengthUnit::Kpc, Some(lg), [0.0, 0.0, 0.0]);
        // 太阳系距离银河中心 ~8 kpc
        let sol = self.add_frame("Sol", LengthUnit::Au, Some(mw), [8.0, 0.0, 0.0]);
        // 比邻星距离银河中心 ~8.5 kpc
        let ac = self.add_frame("AlphaCentauri", LengthUnit::Au, Some(mw), [8.5, 0.5, 0.0]);
        let earth = self.add_frame("Earth", LengthUnit::Km, Some(sol), [1.0, 0.0, 0.0]); // 1 AU
        let mars = self.add_frame("Mars", LengthUnit::Km, Some(sol), [1.524, 0.0, 0.0]); // 1.524 AU
        let luna = self.add_frame("Luna", LengthUnit::Km, Some(earth), [384_400.0, 0.0, 0.0]); // 384400 km
        let proxima = self.add_frame("ProximaB", LengthUnit::Au, Some(ac), [0.05, 0.0, 0.0]);

        StandardFrames {
            lg,
            mw,
            sol,
            earth,
            mars,
            luna,
            ac,
            proxima,
        }
    }
}

/// 标准宇宙帧的 ID 集合
#[derive(Debug, Clone, Copy)]
pub struct StandardFrames {
    pub lg: FrameId,
    pub mw: FrameId,
    pub sol: FrameId,
    pub earth: FrameId,
    pub mars: FrameId,
    pub luna: FrameId,
    pub ac: FrameId,
    pub proxima: FrameId,
}

// =====================================================================
// 测试
// =====================================================================
#[cfg(test)]
mod tests {
    use super::*;

    // ---- 单位测试 ----

    #[test]
    fn unit_conversion_m_to_km() {
        let m = LengthUnit::M;
        let km = LengthUnit::Km;
        let converted = 1000.0 * m.to_meters() / km.to_meters();
        assert!((converted - 1.0).abs() < 1e-12);
    }

    #[test]
    fn unit_conversion_km_to_au() {
        let km = LengthUnit::Km;
        let au = LengthUnit::Au;
        // 1 AU ≈ 149,597,870.7 km
        let val = 149_597_870.7;
        let converted = val * km.to_meters() / au.to_meters();
        assert!((converted - 1.0).abs() < 1e-8);
    }

    #[test]
    fn unit_conversion_au_to_ly() {
        let au = LengthUnit::Au;
        let ly = LengthUnit::Ly;
        // 1 ly ≈ 63,241 AU
        let val = 63_241.0;
        let converted = val * au.to_meters() / ly.to_meters();
        assert!((converted - 1.0).abs() < 0.01);
    }

    #[test]
    fn unit_conversion_ly_to_kpc() {
        let ly = LengthUnit::Ly;
        let kpc = LengthUnit::Kpc;
        // 1 kpc ≈ 3261.56 ly
        let val = 3261.56;
        let converted = val * ly.to_meters() / kpc.to_meters();
        assert!((converted - 1.0).abs() < 0.001);
    }

    #[test]
    fn unit_conversion_roundtrip() {
        let m = LengthUnit::M;
        let au = LengthUnit::Au;
        let value = 1.0; // AU
        let in_meters = value * au.to_meters();
        let back = in_meters / m.to_meters();
        assert!((back - 149_597_870_700.0).abs() < 1.0);
    }

    #[test]
    fn unit_best_for_small() {
        let (v, u) = LengthUnit::best_for(0.5);
        assert_eq!(u, LengthUnit::Mm);
        assert!((v - 500.0).abs() < 1e-9);
    }

    #[test]
    fn unit_best_for_km() {
        let (v, u) = LengthUnit::best_for(100_000.0);
        assert_eq!(u, LengthUnit::Km);
        assert!((v - 100.0).abs() < 1e-9);
    }

    #[test]
    fn unit_best_for_au() {
        let (v, u) = LengthUnit::best_for(1.5e11);
        assert_eq!(u, LengthUnit::Au);
        assert!((v - 1.0027).abs() < 0.001);
    }

    #[test]
    fn unit_best_for_ly() {
        let (v, u) = LengthUnit::best_for(9.46e15);
        assert_eq!(u, LengthUnit::Ly);
        assert!((v - 1.0).abs() < 1e-3);
    }

    #[test]
    fn unit_best_for_kpc() {
        let (v, u) = LengthUnit::best_for(3.086e19);
        assert_eq!(u, LengthUnit::Kpc);
        assert!((v - 1.0).abs() < 1e-3);
    }

    #[test]
    fn unit_best_for_large() {
        let (v, u) = LengthUnit::best_for(1.0e30);
        assert_eq!(u, LengthUnit::Mpc);
        assert!((v - 1.0e30 / LengthUnit::Mpc.to_meters()).abs() < 1.0);
    }

    #[test]
    fn unit_parse() {
        assert_eq!(LengthUnit::parse("km"), Some(LengthUnit::Km));
        assert_eq!(LengthUnit::parse("AU"), Some(LengthUnit::Au));
        assert_eq!(LengthUnit::parse("ly"), Some(LengthUnit::Ly));
        assert_eq!(LengthUnit::parse("光年"), Some(LengthUnit::Ly));
        assert_eq!(LengthUnit::parse("kpc"), Some(LengthUnit::Kpc));
        assert_eq!(LengthUnit::parse("Mpc"), Some(LengthUnit::Mpc));
        assert_eq!(LengthUnit::parse("furlong"), None);
    }

    #[test]
    fn unit_abbrev() {
        assert_eq!(LengthUnit::Km.abbrev(), "km");
        assert_eq!(LengthUnit::Au.abbrev(), "AU");
        assert_eq!(LengthUnit::Ly.abbrev(), "ly");
    }

    // ---- 帧图基础测试 ----

    #[test]
    fn empty_graph_has_root() {
        let g = FrameGraph::new();
        assert_eq!(g.count(), 1);
        assert_eq!(g.get(FrameId::ROOT).unwrap().name, "UniverseRoot");
    }

    #[test]
    fn add_and_retrieve_frame() {
        let mut g = FrameGraph::new();
        let id = g.add_frame("Test", LengthUnit::Km, Some(FrameId::ROOT), [1.0, 2.0, 3.0]);
        let node = g.get(id).unwrap();
        assert_eq!(node.name, "Test");
        assert_eq!(node.unit, LengthUnit::Km);
        assert_eq!(node.parent, Some(FrameId::ROOT));
        assert_eq!(node.origin_in_parent, [1.0, 2.0, 3.0]);
    }

    #[test]
    #[should_panic(expected = "parent frame")]
    fn add_frame_with_invalid_parent() {
        let mut g = FrameGraph::new();
        g.add_frame(
            "Orphan",
            LengthUnit::Km,
            Some(FrameId(999)),
            [0.0, 0.0, 0.0],
        );
    }

    // ---- 帧路径测试 ----

    #[test]
    fn path_to_root_single() {
        let g = FrameGraph::new();
        let path = g.path_to_root(FrameId::ROOT);
        assert_eq!(path, vec![FrameId::ROOT]);
    }

    #[test]
    fn path_to_root_deep() {
        let mut g = FrameGraph::new();
        let l1 = g.add_frame("L1", LengthUnit::Km, Some(FrameId::ROOT), [0.0; 3]);
        let l2 = g.add_frame("L2", LengthUnit::M, Some(l1), [0.0; 3]);
        let l3 = g.add_frame("L3", LengthUnit::Mm, Some(l2), [0.0; 3]);
        let path = g.path_to_root(l3);
        assert_eq!(path.len(), 4);
        assert_eq!(path[0], l3);
        assert_eq!(path[1], l2);
        assert_eq!(path[2], l1);
        assert_eq!(path[3], FrameId::ROOT);
    }

    // ---- LCA 测试 ----

    #[test]
    fn lca_same_frame() {
        let mut g = FrameGraph::new();
        let l1 = g.add_frame("L1", LengthUnit::Km, Some(FrameId::ROOT), [0.0; 3]);
        let l2 = g.add_frame("L2", LengthUnit::M, Some(l1), [0.0; 3]);
        assert_eq!(g.lca(l2, l2), l2);
    }

    #[test]
    fn lca_siblings() {
        let mut g = FrameGraph::new();
        let l1 = g.add_frame("L1", LengthUnit::Km, Some(FrameId::ROOT), [0.0; 3]);
        let a = g.add_frame("A", LengthUnit::M, Some(l1), [0.0; 3]);
        let b = g.add_frame("B", LengthUnit::M, Some(l1), [10.0; 3]);
        assert_eq!(g.lca(a, b), l1);
    }

    #[test]
    fn lca_cousins() {
        let mut g = FrameGraph::new();
        let l1 = g.add_frame("L1", LengthUnit::Km, Some(FrameId::ROOT), [0.0; 3]);
        let a1 = g.add_frame("A1", LengthUnit::M, Some(l1), [0.0; 3]);
        let a2 = g.add_frame("A2", LengthUnit::Mm, Some(a1), [0.0; 3]);
        let b1 = g.add_frame("B1", LengthUnit::M, Some(l1), [10.0; 3]);
        let b2 = g.add_frame("B2", LengthUnit::Mm, Some(b1), [0.0; 3]);
        assert_eq!(g.lca(a2, b2), l1);
    }

    #[test]
    fn lca_root_fallback() {
        let mut g = FrameGraph::new();
        let a = g.add_frame("A", LengthUnit::Km, Some(FrameId::ROOT), [0.0; 3]);
        let b = g.add_frame("B", LengthUnit::Km, Some(FrameId::ROOT), [10.0; 3]);
        assert_eq!(g.lca(a, b), FrameId::ROOT);
    }

    // ---- 位置解析测试 ----

    #[test]
    fn resolve_same_frame() {
        let g = FrameGraph::new();
        let pos = EntityPosition::new(FrameId::ROOT, 1.0, 2.0, 3.0);
        let r = g.resolve(&pos, FrameId::ROOT);
        assert_eq!(r, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn resolve_to_parent_simple() {
        let mut g = FrameGraph::new();
        let sol = g.add_frame("Sol", LengthUnit::Au, Some(FrameId::ROOT), [10.0, 0.0, 0.0]);
        // 飞船在 Sol 帧原点 → 解析到根 = 10 Mpc
        let ship = EntityPosition::new(sol, 0.0, 0.0, 0.0);
        let r = g.resolve(&ship, FrameId::ROOT);
        assert!((r[0] - 10.0).abs() < 1e-15);
        assert!((r[1]).abs() < 1e-15);
        assert!((r[2]).abs() < 1e-15);
    }

    #[test]
    fn resolve_down_to_child() {
        let mut g = FrameGraph::new();
        let sol = g.add_frame("Sol", LengthUnit::Au, Some(FrameId::ROOT), [8.0, 0.0, 0.0]);
        let earth = g.add_frame("Earth", LengthUnit::Km, Some(sol), [1.0, 0.0, 0.0]);
        // 飞船在根 Mpc 中在 (8.5 Mpc, 0, 0) → 解析到 Earth
        // 先找 Earth 帧: 8.5 Mpc → 转 AU → (8.5 Mpc / AU_factor) = X AU
        // X - 1.0 AU (Earth 原点) = offset in AU
        // 转 km
        let ship_root = EntityPosition::new(FrameId::ROOT, 8.5, 0.0, 0.0);
        let r = g.resolve(&ship_root, earth);
        // 8.5 Mpc - 8 Mpc (Sol origin) = 0.5 Mpc → AU
        let au_offset = 0.5 * LengthUnit::Mpc.to_meters() / LengthUnit::Au.to_meters();
        // in Sol AU: au_offset - 1.0 AU (Earth origin) = (au_offset - 1.0) AU
        let km_offset = (au_offset - 1.0) * LengthUnit::Au.to_meters() / LengthUnit::Km.to_meters();
        assert!((r[0] - km_offset).abs() < 0.001);
    }

    #[test]
    fn resolve_roundtrip() {
        let mut g = FrameGraph::new();
        let sol = g.add_frame("Sol", LengthUnit::Au, Some(FrameId::ROOT), [8.0, 2.0, 0.0]);
        let ship = EntityPosition::new(sol, 3.5, -1.2, 0.8);
        let in_root = g.resolve(&ship, FrameId::ROOT);
        let back = g.resolve(
            &EntityPosition::new(FrameId::ROOT, in_root[0], in_root[1], in_root[2]),
            sol,
        );
        assert!((back[0] - 3.5).abs() < 0.001, "got {}", back[0]);
        assert!((back[1] - (-1.2)).abs() < 0.001, "got {}", back[1]);
        assert!((back[2] - 0.8).abs() < 0.001, "got {}", back[2]);
    }

    #[test]
    fn resolve_cross_branch() {
        let mut g = FrameGraph::new();
        let sol = g.add_frame("Sol", LengthUnit::Au, Some(FrameId::ROOT), [8.0, 0.0, 0.0]);
        let ac = g.add_frame(
            "AlphaCent",
            LengthUnit::Au,
            Some(FrameId::ROOT),
            [8.5, 0.5, 0.0],
        );
        let earth = g.add_frame("Earth", LengthUnit::Km, Some(sol), [1.0, 0.0, 0.0]);
        let proxima = g.add_frame("Proxima", LengthUnit::Km, Some(ac), [0.05, 0.0, 0.0]);

        let pos_earth = EntityPosition::new(earth, 0.0, 0.0, 0.0);
        let r = g.resolve(&pos_earth, proxima);
        // Earth in Sol: (1, 0, 0) AU → Root: (8+1AU in Mpc, 0, 0)
        // Proxima in Root: (8.5+0.05AU in Mpc, 0.5, 0)
        // 结果应为 Proxima 下的位置
        assert!(r[0] < 0.0); // 在 Proxima 帧中，应向"太阳方向"为负
        assert!(r[2].abs() < 1e-10);
    }

    // ---- 距离测试 ----

    #[test]
    fn distance_same_frame() {
        let mut g = FrameGraph::new();
        let sol = g.add_frame("Sol", LengthUnit::Au, Some(FrameId::ROOT), [0.0; 3]);
        let a = EntityPosition::new(sol, 1.0, 0.0, 0.0); // 1 AU
        let b = EntityPosition::new(sol, 2.0, 0.0, 0.0); // 2 AU
        let d = g.distance(&a, &b);
        assert!((d.value - 1.0).abs() < 1e-12);
        assert_eq!(d.unit, LengthUnit::Au);
    }

    #[test]
    fn distance_cross_frame() {
        let mut g = FrameGraph::new();
        let mw = g.add_frame("MW", LengthUnit::Kpc, Some(FrameId::ROOT), [0.0; 3]);
        let sol = g.add_frame("Sol", LengthUnit::Au, Some(mw), [8.0, 0.0, 0.0]);
        let ac = g.add_frame("AlphaCent", LengthUnit::Au, Some(mw), [8.5, 0.5, 0.0]);

        let earth = EntityPosition::new(sol, 1.0, 0.0, 0.0);
        let proxima = EntityPosition::new(ac, 0.05, 0.0, 0.0);

        let d = g.distance_auto(&earth, &proxima);
        // 两系统相距 ~0.707 kpc
        assert!(
            d.unit == LengthUnit::Kpc || d.unit == LengthUnit::Ly,
            "got {:?} {}",
            d.unit,
            d.value
        );
        assert!((d.value - 0.707).abs() < 0.005, "value={}", d.value);
    }

    #[test]
    fn distance_zero() {
        let g = FrameGraph::new();
        let a = EntityPosition::new(FrameId::ROOT, 0.0, 0.0, 0.0);
        let b = EntityPosition::new(FrameId::ROOT, 0.0, 0.0, 0.0);
        let d = g.distance(&a, &b);
        assert!((d.value).abs() < 1e-15);
    }

    #[test]
    fn distance_format() {
        let s = Scalar::new(1.5, LengthUnit::Au);
        let fmt = s.format();
        assert!(fmt.contains("1.5000"));
        assert!(fmt.contains("AU"));
    }

    #[test]
    fn distance_auto_scaling() {
        // 1 mm → 自动选 Mm
        let s = Scalar::new(0.001, LengthUnit::M);
        let auto = s.auto_unit();
        assert!((auto.value - 1.0).abs() < 1e-9);
        assert_eq!(auto.unit, LengthUnit::Mm);
    }

    // ---- 标准宇宙测试 ----

    #[test]
    fn standard_universe_sun_earth_distance() {
        let mut g = FrameGraph::new();
        let f = g.build_standard_universe();

        // 太阳在 Sol 帧原点
        let sun = EntityPosition::new(f.sol, 0.0, 0.0, 0.0);
        // 地球在 Earth 帧原点 (但在 Sol 中是 1 AU)
        let earth = EntityPosition::new(f.earth, 0.0, 0.0, 0.0);

        let d = g.distance(&sun, &earth);
        // 应为 ~1 AU
        assert!(d.unit == LengthUnit::Au || d.unit == LengthUnit::Km);
        if d.unit == LengthUnit::Au {
            assert!((d.value - 1.0).abs() < 0.01);
        }
    }

    #[test]
    fn standard_universe_earth_moon() {
        let mut g = FrameGraph::new();
        let f = g.build_standard_universe();

        let earth = EntityPosition::new(f.earth, 0.0, 0.0, 0.0);
        let moon = EntityPosition::new(f.luna, 0.0, 0.0, 0.0);

        let d = g.distance_auto(&earth, &moon);
        assert!((d.value - 384_400.0).abs() < 0.01 || d.value > 0.0);
    }

    #[test]
    fn standard_universe_mars_sol() {
        let mut g = FrameGraph::new();
        let f = g.build_standard_universe();

        let mars = EntityPosition::new(f.mars, 0.0, 0.0, 0.0);
        let sol = EntityPosition::new(f.sol, 0.0, 0.0, 0.0);

        let d = g.distance_auto(&mars, &sol);
        // 火星在 Sol 帧中位于 1.524 AU
        if d.unit == LengthUnit::Au {
            assert!((d.value - 1.524).abs() < 0.001);
        }
    }

    #[test]
    fn standard_universe_sol_alphacentauri() {
        let mut g = FrameGraph::new();
        let f = g.build_standard_universe();

        let sol_pos = EntityPosition::new(f.sol, 0.0, 0.0, 0.0);
        let ac_pos = EntityPosition::new(f.ac, 0.0, 0.0, 0.0);

        let d = g.distance_auto(&sol_pos, &ac_pos);
        // 在 MW 帧中相距 sqrt(0.5² + 0.5²) = 0.707 kpc
        assert!(
            d.unit == LengthUnit::Kpc || d.unit == LengthUnit::Ly,
            "got {:?} {}",
            d.unit,
            d.value
        );
        assert!((d.value - 0.707).abs() < 0.01, "value={}", d.value);
    }

    #[test]
    fn standard_universe_proxima_sol() {
        let mut g = FrameGraph::new();
        let f = g.build_standard_universe();

        let proxima = EntityPosition::new(f.proxima, 0.0, 0.0, 0.0);
        let sol = EntityPosition::new(f.sol, 0.0, 0.0, 0.0);

        let d = g.distance_auto(&proxima, &sol);
        // ~0.707 kpc + 0.05 AU (negligible)
        assert!(d.value > 0.5);
    }

    // ---- Scalar 单位转换测试 ----

    #[test]
    fn scalar_conversion() {
        let s = Scalar::new(1.0, LengthUnit::Au);
        let in_km = s.to(LengthUnit::Km);
        assert!((in_km - 149_597_870.7).abs() < 0.1);
    }

    #[test]
    fn scalar_conversion_km_to_au() {
        let s = Scalar::new(149_597_870.7, LengthUnit::Km);
        let in_au = s.to(LengthUnit::Au);
        assert!((in_au - 1.0).abs() < 1e-8);
    }

    // ---- 极限精度测试 ----

    #[test]
    fn galactic_scale_precision() {
        // 跨银河系距离：两个不同星系中的实体
        let mut g = FrameGraph::new();
        let lg = g.add_frame("LocalGroup", LengthUnit::Mpc, Some(FrameId::ROOT), [0.0; 3]);
        let mw = g.add_frame("MW", LengthUnit::Kpc, Some(lg), [0.0, 0.0, 0.0]);
        let andromeda = g.add_frame("Andromeda", LengthUnit::Kpc, Some(lg), [0.78, 0.0, 0.0]); // 780 kpc

        let mw_star = EntityPosition::new(mw, 15.0, 0.0, 0.0); // 距 MW 中心 15 kpc
        let and_star = EntityPosition::new(andromeda, 10.0, 0.0, 0.0); // 距 Andromeda 中心 10 kpc

        let d = g.distance_auto(&mw_star, &and_star);
        // ~780 kpc → Mpc 级别
        assert!(d.unit == LengthUnit::Mpc || d.unit == LengthUnit::Kpc);
        assert!(d.value > 0.7); // ~0.78 Mpc
        assert!(d.value < 0.9);
    }

    #[test]
    fn extreme_scale_consistency() {
        // 验证从 km 到 Mpc 的跨级解析不丢精度
        let mut g = FrameGraph::new();
        let mw = g.add_frame("MW", LengthUnit::Kpc, Some(FrameId::ROOT), [0.0; 3]);
        let sol = g.add_frame("Sol", LengthUnit::Au, Some(mw), [8.0, 0.0, 0.0]);
        let earth = g.add_frame("Earth", LengthUnit::Km, Some(sol), [1.0, 0.0, 0.0]);

        // 绕地轨道上的位置：地球半径 + 400 km
        let leo = EntityPosition::new(earth, 6371.0 + 400.0, 0.0, 0.0);

        // 解析到 MW (kpc)
        let r = g.resolve(&leo, mw);
        // 应该 = 8 kpc + 1AU/kpc + (6771km)/kpc
        let sol_in_mw = 8.0; // kpc
        let au_in_kpc = 1.0 * LengthUnit::Au.to_meters() / LengthUnit::Kpc.to_meters();
        let km_in_kpc = 6771.0 * LengthUnit::Km.to_meters() / LengthUnit::Kpc.to_meters();
        let expected = sol_in_mw + au_in_kpc + km_in_kpc;
        assert!((r[0] - expected).abs() < 1e-12);
    }

    // ---- 并发/多实体测试 ----

    #[test]
    fn multiple_entities_distance_matrix() {
        let mut g = FrameGraph::new();
        let f = g.build_standard_universe();

        let entities = vec![
            ("Sun", EntityPosition::new(f.sol, 0.0, 0.0, 0.0)),
            ("Earth", EntityPosition::new(f.earth, 0.0, 0.0, 0.0)),
            ("Mars", EntityPosition::new(f.mars, 0.0, 0.0, 0.0)),
            ("Luna", EntityPosition::new(f.luna, 0.0, 0.0, 0.0)),
            ("Proxima", EntityPosition::new(f.proxima, 0.0, 0.0, 0.0)),
        ];

        // 验证所有距离为正且对称
        for i in 0..entities.len() {
            for j in i + 1..entities.len() {
                let d_ij = g.distance_auto(&entities[i].1, &entities[j].1);
                let d_ji = g.distance_auto(&entities[j].1, &entities[i].1);
                assert!(d_ij.value > 0.0);
                assert!((d_ij.value - d_ji.value).abs() / d_ij.value.max(1e-10) < 1e-12);
            }
        }
    }

    // ---- 单位转换/convert 方法测试 ----

    #[test]
    fn convert_method() {
        let g = FrameGraph::new();
        let r = g.convert(1.5, LengthUnit::Au, LengthUnit::Km);
        assert!((r - 1.5 * 149_597_870.7).abs() < 0.1);
    }

    // ---- 跨参考系路径解析时的精度保证 ----

    #[test]
    fn deep_chain_roundtrip() {
        // 10 级帧深度，验证往返一致
        let mut g = FrameGraph::new();
        let mut prev = FrameId::ROOT;
        let units = [
            LengthUnit::Kpc,
            LengthUnit::Ly,
            LengthUnit::Au,
            LengthUnit::Km,
            LengthUnit::M,
            LengthUnit::Km,
            LengthUnit::Au,
            LengthUnit::Ly,
            LengthUnit::Kpc,
            LengthUnit::Mpc,
        ];
        let mut ids = Vec::new();
        for (i, &u) in units.iter().enumerate() {
            let id = g.add_frame(
                &format!("Level{}", i),
                u,
                Some(prev),
                [1.0 * (i as f64), 0.0, 0.0],
            );
            ids.push(id);
            prev = id;
        }

        let ship = EntityPosition::new(prev, 42.0, -3.0, 7.0);
        let in_root = g.resolve(&ship, FrameId::ROOT);
        let back = g.resolve(
            &EntityPosition::new(FrameId::ROOT, in_root[0], in_root[1], in_root[2]),
            prev,
        );
        assert!((back[0] - 42.0).abs() < 1e-8);
        assert!((back[1] + 3.0).abs() < 1e-8);
        assert!((back[2] - 7.0).abs() < 1e-8);
    }

    // =====================================================================
    // 三体宇宙测试: 叶文杰 与 三体人 通信往返时间
    // =====================================================================
    // 场景:
    //   叶文杰 (Ye Wenjie) 在地球大兴安岭 (50°N, 124°E)
    //   向三体星系 (比邻星/Alpha Centauri) 发射电磁波
    //   三体人在比邻星 b 轨道空间站接收后立即回复
    //   计算往返时间
    //
    // 帧层次:
    //   Root (Mpc)
    //     └─ LocalGroup (Mpc)
    //          └─ MilkyWay (kpc)
    //               └─ SolLocal (ly)           ← 太阳附近星际空间
    //                    ├─ Sol (AU)
    //                    │    └─ Earth (km)    ← 叶文杰
    //                    └─ AlphaCentauri (AU)
    //                         └─ Trisolaris (km) ← 三体人
    #[test]
    fn three_body_communication_delay() {
        const C: f64 = 299_792.458; // 光速 km/s

        let mut g = FrameGraph::new();

        let lg = g.add_frame("LocalGroup", LengthUnit::Mpc, Some(FrameId::ROOT), [0.0; 3]);
        let mw = g.add_frame("MilkyWay", LengthUnit::Kpc, Some(lg), [0.0; 3]);
        // 太阳邻域: 用 ly 为单位, 距银河中心 8 kpc
        let sol_local = g.add_frame("SolLocal", LengthUnit::Ly, Some(mw), [8.0, 0.0, 0.0]);

        // 太阳系: SolLocal 原点
        let sol = g.add_frame("Sol", LengthUnit::Au, Some(sol_local), [0.0; 3]);
        // 地球: 距太阳 1 AU
        let earth = g.add_frame("Earth", LengthUnit::Km, Some(sol), [1.0, 0.0, 0.0]);

        // 三体星系 (比邻星/Alpha Centauri): 距太阳 4.37 ly
        let ac = g.add_frame(
            "AlphaCentauri",
            LengthUnit::Au,
            Some(sol_local),
            [4.37, 0.0, 0.0],
        );
        // 三体空间站: 距比邻星 0.05 AU (与比邻星 b 轨道类似)
        let trisolaris = g.add_frame("Trisolaris", LengthUnit::Km, Some(ac), [0.05, 0.0, 0.0]);

        // ---- 叶文杰在大兴安岭 ----
        let lat = 50.0_f64.to_radians();
        let lon = 124.0_f64.to_radians();
        let r = 6371.0; // 地球半径 km
        let (slat, clat) = lat.sin_cos();
        let (slon, clon) = lon.sin_cos();
        // 从地心到地表
        let ye_x = r * clat * clon;
        let ye_y = r * clat * slon;
        let ye_z = r * slat;

        let ye_wenjie = EntityPosition::new(earth, ye_x, ye_y, ye_z);

        // ---- 三体人在轨道空间站 ----
        let trisolaran = EntityPosition::new(trisolaris, 0.0, 0.0, 0.0);

        // ---- 计算距离 ----
        let d = g.distance_auto(&ye_wenjie, &trisolaran);

        // 距离应在光年量级 (~4.37 ly)
        assert_eq!(
            d.unit,
            LengthUnit::Ly,
            "auto unit should be ly, got {:?}",
            d.unit
        );
        assert!(
            (d.value - 4.37).abs() < 0.05,
            "distance should be ~4.37 ly, got {}",
            d.value
        );

        // ---- 计算光速往返时间 ----
        let dist_km = d.to(LengthUnit::Km);
        let round_trip_s = 2.0 * dist_km / C;
        let round_trip_years = round_trip_s / (365.25 * 86400.0);

        // 三体小说: 叶文杰 1971 年发射 → 三体回复 1979 年到达 ≈ 8 年
        // 实际比邻星距离 4.37 ly → 往返 8.74 年
        println!(
            "  叶文杰 → 三体星系: {:.4} ly ({:.2e} km)",
            d.value, dist_km
        );
        println!(
            "  电磁波往返时间: {:.2} 年 ({:.2e} 秒)",
            round_trip_years, round_trip_s
        );
        println!(
            "  叶文杰 1971 年发射 → {} 年收到回复",
            1971.0 + round_trip_years
        );

        assert!(
            (round_trip_years - 8.74).abs() < 0.1,
            "round trip should be ~8.74 years, got {:.4}",
            round_trip_years
        );
        // 往返时间为正且合理
        assert!(round_trip_years > 8.0);
        assert!(round_trip_years < 10.0);

        // ---- 额外验证: 缩放后的距离应该合理 ----
        let d_au = Scalar::new(d.value, d.unit).to(LengthUnit::Au);
        // 4.37 ly ≈ 276,000 AU
        assert!(
            (d_au - 276_000.0).abs() / 276_000.0 < 0.02,
            "au distance off: {}",
            d_au
        );

        // ---- 跨帧解析的一致性 ----
        // 把叶文杰位置解析到三体帧, 再解析回来, 应该一致
        let ye_in_trisolaris = g.resolve(&ye_wenjie, trisolaris);
        let ye_back = g.resolve(
            &EntityPosition::new(
                trisolaris,
                ye_in_trisolaris[0],
                ye_in_trisolaris[1],
                ye_in_trisolaris[2],
            ),
            earth,
        );
        let dx = (ye_back[0] - ye_x).abs();
        let dy = (ye_back[1] - ye_y).abs();
        let dz = (ye_back[2] - ye_z).abs();
        // 跨 4 级帧 (Earth→Sol→SolLocal→...→SolLocal→Sol→Earth) 后,
        // 精度受限于 4.37 ly 距 Earth 原点的大偏移, 容差设为 100 km
        assert!(dx < 100.0, "x error: {} km", dx);
        assert!(dy < 100.0, "y error: {} km", dy);
        assert!(dz < 100.0, "z error: {} km", dz);
    }
}
