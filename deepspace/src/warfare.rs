//! 作战模拟模块
//!
//! 提供空战交战管理、命中概率、杀伤评估、交战阶段状态机。

// =====================================================================
// 交战阶段
// =====================================================================

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EngagementPhase {
    /// 搜索中
    Search,
    /// 探测到目标但未识别
    Detected,
    /// 已识别并跟踪
    Identified,
    /// 已进入交战范围
    Engaged,
    /// BVR 导弹已发射
    BvrEngagement,
    /// 进入视距内格斗
    WithinVisualRange,
    /// 脱离
    Disengaged,
    /// 结束
    Terminated,
}

impl EngagementPhase {
    pub fn name(&self) -> &'static str {
        match self {
            EngagementPhase::Search => "Search",
            EngagementPhase::Detected => "Detected",
            EngagementPhase::Identified => "Identified",
            EngagementPhase::Engaged => "Engaged",
            EngagementPhase::BvrEngagement => "BVR",
            EngagementPhase::WithinVisualRange => "WVR",
            EngagementPhase::Disengaged => "Disengaged",
            EngagementPhase::Terminated => "Terminated",
        }
    }
}

// =====================================================================
// 杀伤概率模型
// =====================================================================

/// 计算单发杀伤概率 (SSPK)
///
/// # Parameters
/// - `cep_m`: 圆概率误差 (m)
/// - `kill_radius_m`: 弹头杀伤半径 (m)
/// - `reliability`: 导弹可靠性 (0~1)
/// - `countermeasure_factor`: 对抗措施影响 (0~1, 1=无影响)
pub fn ssppk(cep_m: f64, kill_radius_m: f64, reliability: f64, countermeasure_factor: f64) -> f64 {
    if cep_m <= 0.0 || kill_radius_m <= 0.0 || reliability <= 0.0 {
        return 0.0;
    }

    // 简化 CEP 命中概率：假设圆形正态分布
    // P_hit ≈ 1 - exp(-ln(2) * (kill_radius / CEP)²)
    let hit_prob = 1.0 - (-(kill_radius_m / cep_m).powi(2) * (2.0_f64.ln())).exp();

    // 考虑可靠性和对抗
    hit_prob * reliability * countermeasure_factor
}

/// 计算累积杀伤概率 (对多次射击)
pub fn cumulative_pk(single_shot_pk: f64, shots: i32) -> f64 {
    if shots <= 0 || single_shot_pk <= 0.0 {
        return 0.0;
    }
    1.0 - (1.0 - single_shot_pk).powi(shots)
}

// =====================================================================
// 空战态势评估
// =====================================================================

/// 角度优势 [-1, 1]: 1=最佳射击位置
pub fn angle_advantage(
    attacker_heading_to_target_deg: f64,
    target_heading_to_attacker_deg: f64,
    attacker_speed: f64,
    target_speed: f64,
) -> f64 {
    // 攻击者指向目标的程度
    let pointing = (1.0 - (attacker_heading_to_target_deg.abs() / 180.0)).max(0.0);
    // 目标背对攻击者的程度
    let target_aspect = (target_heading_to_attacker_deg.abs() / 180.0).max(0.0);

    let aspect_score = 1.0 - target_aspect;
    let speed_ratio = (attacker_speed / target_speed.max(1.0)).clamp(0.5, 2.0);
    let speed_score = (speed_ratio - 1.0) * 0.5;

    (pointing * 0.5 + aspect_score * 0.3 + speed_score * 0.2).clamp(-1.0, 1.0)
}

/// 能量优势 [-1, 1]
pub fn energy_advantage(
    attacker_alt_m: f64,
    target_alt_m: f64,
    attacker_speed: f64,
    target_speed: f64,
) -> f64 {
    let alt_diff = (attacker_alt_m - target_alt_m) / 1000.0;
    let alt_score = alt_diff.clamp(-3.0, 3.0) / 3.0;

    let speed_diff = (attacker_speed - target_speed) / 100.0;
    let speed_score = speed_diff.clamp(-3.0, 3.0) / 3.0;

    (alt_score * 0.5 + speed_score * 0.5).clamp(-1.0, 1.0)
}

// =====================================================================
// 测试
// =====================================================================
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sspk_calculation() {
        let pk = ssppk(10.0, 12.0, 0.9, 1.0);
        assert!(pk > 0.5 && pk <= 1.0);
    }

    #[test]
    fn cumulative_pk_increases() {
        let single = 0.5;
        let cum2 = cumulative_pk(single, 2);
        assert!(cum2 > single);
        assert!((cum2 - 0.75).abs() < 0.01);
    }

    #[test]
    fn angle_advantage_from_behind() {
        // 从正后方攻击
        let adv = angle_advantage(0.0, 180.0, 300.0, 250.0);
        assert!(adv > 0.0);
    }

    #[test]
    fn energy_advantage_higher() {
        let adv = energy_advantage(10000.0, 5000.0, 300.0, 300.0);
        assert!(adv > 0.0);
    }
}
