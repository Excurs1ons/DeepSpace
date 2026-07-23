#pragma once
// ============================================================================
// DeepSpaceMissionPlanner.h
// ----------------------------------------------------------------------------
// 深空任务链规划器 (探索闭环核心): 地月转移 / 月球捕获 / 着陆 状态机。
//
// 设计目标:
//   给定地球(earth)与月球(moon)两个天体, 规划一条 "停泊轨道 -> 跨月转移 ->
//   月球捕获(LOI) -> 着陆" 的任务链, 并提供基于遥测(高度/速度)的相位状态机
//   AdvancePhase(), 保证任务能走到 Completed 或 Failed 闭环。
//
// 依赖契约 (其他代理并发实现, 运行时真实链接, 签名不可改):
//   - LambertSolver::Hohmann(r1,r2,mu) -> HohmannTransfer{deltaV1,deltaV2,
//        totalDeltaV,transferTime; bool valid}   (霍曼转移双脉冲)
//   - LambertSolver::Solve(r1,r2,tof,mu,shortWay=true) -> LambertSolution{
//        Vec3d departureVelocity; Vec3d arrivalVelocity; double transferTime;
//        bool valid}                              (兰伯特普遍转移)
//   - Integrators::PropagateTwoBody(pos,vel,mu,t0,tEnd,dt) -> vector<double>(6维)
//   - Planet: GetMass(), GetRadius()
//   - Vec3d: Vec3d(x,y,z); .x/.y/.z; 算术; .Length(); Vec3d::Dot(a,b);
//        Vec3d::Cross(a,b)
//
// 物理公式 (全部 double):
//   * 地球引力参数  muEarth = G * earth.GetMass()  (≈ 3.986e14 m^3/s^2)
//   * 圆轨道速度    vCirc   = sqrt(mu / r)
//   * 霍曼首次脉冲  deltaV1 = 转移轨道近地点速度 - 停泊圆轨速度
//   * 月球捕获脉冲  deltaV  = |vArrival - vTarget|   (到达速度减目标圆轨速度)
//   * 兰伯特出发脉冲 deltaV = |departureVelocity - vCirc_vec|, 其中 vCirc_vec
//                          沿 departure 方向, 模为 sqrt(mu/rPark)
//
// 退化保护: 任何半径/时间 <= 0 的入参一律返回 success=false, 进入 Failed。
// 纯头文件、内联实现; 不引入除标准库与既有契约头之外的依赖。
// ============================================================================

#include <string>
#include <cmath>
#include <vector>

#include "../physics/Integrators.h"   // 引入 Vec3d(=Mock::Vec3d), Integrators, Constants
#include "../environment/Planet.h"     // 引入 Planet(GetMass/GetRadius)
#include "../physics/LambertSolver.h"  // 复用真实 LambertSolution / HohmannTransfer / LambertSolver 定义（单一真相，避免重复定义）

namespace DeepSpace {

// ===========================================================================
// 任务相位枚举
// ===========================================================================
enum class MissionPhase {
    PreLaunch,            // 发射前
    EarthOrbit,           // 地球停泊轨道
    TransLunarInjection,  // 跨月注入 (TLI) 点火后
    CoastToMoon,          // 向月球滑行
    LunarOrbitInsertion,  // 月球捕获 (LOI) 阶段
    Landing,              // 着陆下降
    Completed,            // 任务成功
    Failed                // 任务失败
};

// ===========================================================================
// 单步规划结果
// ===========================================================================
struct MissionStepResult {
    bool         success;   // 该步规划是否成功
    double       deltaV;    // 所需速度增量 (m/s)
    MissionPhase nextPhase; // 建议进入的下一相位
    std::string  message;   // 人类可读说明
};

// ===========================================================================
// DeepSpaceMissionPlanner: 地月转移/捕获/着陆规划与状态机
// ===========================================================================
class DeepSpaceMissionPlanner {
public:
    // 构造: 保存地球/月球, 预计算地球引力参数。
    DeepSpaceMissionPlanner(const Planet& earth, const Planet& moon)
        : m_Earth(earth),
          m_Moon(moon),
          m_MuEarth(Constants::G * earth.GetMass()) {}

    // --- 规划接口 (纯计算, 不修改状态) ---

    // 跨月注入 (TLI): 用霍曼转移计算从停泊轨道半径 rPark 到月球轨道半径 rMoon
    // 的首次脉冲 deltaV1 与转移时间。返回 deltaV = deltaV1。
    MissionStepResult PlanTransLunarInjection(double rPark, double rMoon) const {
        if (rPark <= 0.0 || rMoon <= 0.0) {
            return {false, 0.0, MissionPhase::Failed,
                    "TLI: invalid orbit radii (rPark/rMoon <= 0)"};
        }
        const HohmannTransfer h = LambertSolver::Hohmann(rPark, rMoon, m_MuEarth);
        if (!h.valid) {
            return {false, 0.0, MissionPhase::Failed,
                    "TLI: Hohmann solution invalid (degenerate geometry)"};
        }
        const double dv = h.deltaV1;
        return {true, dv, MissionPhase::CoastToMoon,
                "TLI planned: deltaV1=" + std::to_string(dv) +
                " m/s, transferTime=" + std::to_string(h.transferTime) + " s"};
    }

    // 月球轨道插入 (LOI): 捕获所需脉冲 = |vArrival - vTarget|。
    MissionStepResult PlanLunarOrbitInsertion(const Vec3d& vArrival,
                                              const Vec3d& vTarget) const {
        const double dv = (vArrival - vTarget).Length();
        // 退化保护: 零矢量长度 (NaN/零) 视为无效。
        if (!(dv > 0.0)) {
            return {false, 0.0, MissionPhase::Failed,
                    "LOI: zero/undefined relative velocity change"};
        }
        return {true, dv, MissionPhase::Landing,
                "LOI planned: deltaV=" + std::to_string(dv) + " m/s"};
    }

    // 兰伯特转移: 调用 LambertSolver::Solve 求普遍转移, 出发脉冲 =
    // |departureVelocity - 当前圆轨速度矢量|, 圆轨速度 = sqrt(mu / rPark)。
    MissionStepResult PlanLambertTransfer(const Vec3d& rEarthOrbit,
                                          const Vec3d& rMoonOrbit,
                                          double tofSeconds) const {
        const double rPark = rEarthOrbit.Length();
        if (rPark <= 0.0 || tofSeconds <= 0.0) {
            return {false, 0.0, MissionPhase::Failed,
                    "Lambert: invalid orbit radius or time-of-flight (<= 0)"};
        }
        const LambertSolution sol =
            LambertSolver::Solve(rEarthOrbit, rMoonOrbit, tofSeconds, m_MuEarth, true);
        if (!sol.valid) {
            return {false, 0.0, MissionPhase::Failed,
                    "Lambert: solver returned invalid solution"};
        }
        const double vCirc = std::sqrt(m_MuEarth / rPark);
        // 当前圆轨速度矢量: 沿 departure 方向、模为 vCirc (切向近似)。
        const Vec3d vCircVec = sol.departureVelocity.Normalized() * vCirc;
        const double dv = (sol.departureVelocity - vCircVec).Length();
        return {true, dv, MissionPhase::CoastToMoon,
                "Lambert transfer planned: deltaV=" + std::to_string(dv) +
                " m/s, tof=" + std::to_string(tofSeconds) + " s"};
    }

    // --- 相位状态机 (遥测驱动, 闭环到 Completed/Failed) ---
    // altitude: 当前所绕天体表面之上的高度 (m); 近月阶段由调用方切换为月球参考系。
    // speed:    相对当前参考天体的速率 (m/s)。
    MissionStepResult AdvancePhase(MissionPhase current,
                                   double altitude, double speed) const {
        const double kParkAltitude      = 1.60e5;   // LEO 停泊轨道高度 (m)
        const double kTLISpeed          = 1.00e4;   // 到达转移速度阈值 (m/s)
        const double kLunarApproachDist = 6.00e7;   // 进入月球影响球距离 (m, 月心)
        const double kLunarOrbitRadius  = m_Moon.GetRadius() + 1.00e5; // 环月轨道半径
        const double kLunarOrbitSpeed   = 2.00e3;   // 环月捕获后速率阈值 (m/s)
        const double kLandingAlt        = 1.0;      // 触月高度阈值 (m)
        const double kTouchdownSpeed    = 5.0;      // 安全着陆速度阈值 (m/s)

        // 终端相位: 保持不变。
        if (current == MissionPhase::Completed)
            return {true, 0.0, MissionPhase::Completed, "Mission completed."};
        if (current == MissionPhase::Failed)
            return {false, 0.0, MissionPhase::Failed, "Mission failed."};

        // 低于任何天体表面且未处于安全着陆 -> 坠毁/再入失败。
        if (altitude <= 0.0 && current != MissionPhase::Landing)
            return {false, 0.0, MissionPhase::Failed,
                    "Altitude <= 0: reentry/crash detected."};

        switch (current) {
            case MissionPhase::PreLaunch:
                if (altitude >= kParkAltitude)
                    return {true, 0.0, MissionPhase::EarthOrbit,
                            "Reached parking orbit."};
                return {true, 0.0, MissionPhase::PreLaunch, "On pad / ascending."};

            case MissionPhase::EarthOrbit:
                if (speed >= kTLISpeed && altitude >= kParkAltitude)
                    return {true, 0.0, MissionPhase::TransLunarInjection,
                            "TLI burn complete, departing Earth."};
                return {true, 0.0, MissionPhase::EarthOrbit, "In Earth parking orbit."};

            case MissionPhase::TransLunarInjection:
                if (altitude >= kLunarApproachDist)
                    return {true, 0.0, MissionPhase::CoastToMoon,
                            "Left Earth vicinity, coasting to Moon."};
                return {true, 0.0, MissionPhase::TransLunarInjection,
                        "Climbing onto trans-lunar trajectory."};

            case MissionPhase::CoastToMoon:
                // 调用方在进入月心参考系后传入月球高度; 接近环月半径即捕获。
                if (altitude <= kLunarOrbitRadius)
                    return {true, 0.0, MissionPhase::LunarOrbitInsertion,
                            "Entered lunar capture zone."};
                return {true, 0.0, MissionPhase::CoastToMoon, "Coasting to Moon."};

            case MissionPhase::LunarOrbitInsertion:
                if (speed <= kLunarOrbitSpeed && altitude <= kLunarOrbitRadius)
                    return {true, 0.0, MissionPhase::Landing,
                            "Captured into lunar orbit, begin descent."};
                return {true, 0.0, MissionPhase::LunarOrbitInsertion,
                        "Braking into lunar orbit."};

            case MissionPhase::Landing:
                if (altitude <= kLandingAlt && speed <= kTouchdownSpeed)
                    return {true, 0.0, MissionPhase::Completed,
                            "Soft landing confirmed."};
                if (altitude <= 0.0 && speed > kTouchdownSpeed)
                    return {false, 0.0, MissionPhase::Failed,
                            "Hard impact: touchdown too fast."};
                return {true, 0.0, MissionPhase::Landing, "Descending to surface."};

            default:
                return {false, 0.0, MissionPhase::Failed, "Unknown phase."};
        }
    }

    // 初始相位。
    MissionPhase GetInitialPhase() const { return MissionPhase::PreLaunch; }

private:
    Planet m_Earth;
    Planet m_Moon;
    double m_MuEarth; // 地球引力参数 mu = G * M_earth
};

} // namespace DeepSpace
