// ===========================================================================
// core_tests.h - 零依赖 mini-test harness + 纯函数黄金测试
// ---------------------------------------------------------------------------
// 说明：
//   * 不引入任何外部测试框架（无 gtest / 无 cmake 全量构建）。
//   * 自带极简 TEST / EXPECT_NEAR / EXPECT_TRUE 宏，main() 跑全套并报告。
//   * 仅做语法检查：
//       cd ~/projects/DeepSpace && g++ -std=c++20 -fsyntax-only -Isrc tests/core_tests.h
//   * 覆盖：轨道要素(OrbitalMechanics)、推进(EnginePart)、O/F 质量比、
//           气动阻力系数(Aerodynamics)、物理积分(PhysicsBody::Update)。
// ===========================================================================
#pragma once

#include <cmath>
#include <cstddef>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

// 被测源码（相对路径，对应 src/ 顶层结构）
#include "../src/environment/Planet.h"
#include "../src/physics/OrbitalElements.h"
#include "../src/physics/Aerodynamics.h"
#include "../src/physics/PhysicsBody.h"
#include "../src/vessel/Part.h"

// ---------------------------------------------------------------------------
// 极简测试 harness
// ---------------------------------------------------------------------------
namespace ds_test {

struct TestCase {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}

inline int g_passed = 0;
inline int g_failed = 0;

struct Registrar {
    Registrar(const std::string& name, std::function<void()> fn) {
        registry().push_back({name, std::move(fn)});
    }
};

inline void report(const std::string& tag, bool ok, const std::string& detail) {
    if (ok) {
        ++g_passed;
        std::cout << "[ PASS ] " << tag << "\n";
    } else {
        ++g_failed;
        std::cout << "[ FAIL ] " << tag << " :: " << detail << "\n";
    }
}

}  // namespace ds_test

#define DS_STR(x) #x
#define DS_XSTR(x) DS_STR(x)

#define DS_TEST(name)                                                       \
    static void name();                                                    \
    static ::ds_test::Registrar DS_CAT(_reg_, __LINE__)(#name, name);      \
    static void name()

#define DS_CAT2(a, b) a##b
#define DS_CAT(a, b) DS_CAT2(a, b)

// EXPECT_NEAR(actual, expected, tol)：|actual-expected| <= tol
#define EXPECT_NEAR(actual, expected, tol)                                  \
    do {                                                                   \
        const double _a = static_cast<double>(actual);                     \
        const double _e = static_cast<double>(expected);                   \
        const double _t = static_cast<double>(tol);                        \
        const bool   _ok = std::isfinite(_a) && std::isfinite(_e) &&       \
                           std::abs(_a - _e) <= _t;                        \
        ::ds_test::report(                                                 \
            std::string(__FILE__ "(" DS_XSTR(__LINE__) ") ") + #actual,    \
            _ok,                                                           \
            std::string("expected ~") + std::to_string(_e) +               \
            " got " + std::to_string(_a) +                                 \
            " (tol " + std::to_string(_t) + ")");                          \
    } while (0)

// EXPECT_TRUE(cond)
#define EXPECT_TRUE(cond)                                                   \
    do {                                                                   \
        const bool _ok = static_cast<bool>(cond);                          \
        ::ds_test::report(                                                 \
            std::string(__FILE__ "(" DS_XSTR(__LINE__) ") ") + #cond,       \
            _ok, std::string("condition was false"));                      \
    } while (0)

// EXPECT_GT / EXPECT_LT
#define EXPECT_GT(a, b) EXPECT_TRUE((a) > (b))
#define EXPECT_LT(a, b) EXPECT_TRUE((a) < (b))

// ===========================================================================
// 测试用例
// ===========================================================================

// ---------------------------------------------------------------------------
// 1) 轨道要素 OrbitalMechanics::CalculateElements
// ---------------------------------------------------------------------------

// 圆轨道（任务给定参数，半径 6771 km ≈ 400 km 高度，圆速 7669 m/s）
DS_TEST(Orbital_Circular_GivenParams) {
    using DeepSpace::Planet;
    using DeepSpace::Atmosphere;
    using DeepSpace::OrbitalMechanics;
    using DeepSpace::Vec3d;

    Planet earth("Earth", 5.9722e24, 6371000.0, Atmosphere(101325, 8500));
    Vec3d pos(0.0, 6.771e6, 0.0);
    Vec3d vel(7669.0, 0.0, 0.0);

    auto el = OrbitalMechanics::CalculateElements(pos, vel, earth);

    // 给定速度下 e≈0（实际 ~9.4e-4，远小于 0.01）
    EXPECT_NEAR(el.eccentricity, 0.0, 0.01);
    EXPECT_TRUE(el.isBound);
    // 远地点恰好为 400 km（速度略低于真圆速 7672.6，此处正是远地点）；
    // 近地点略低（~387.2 km）。7669 与 6771 km 半径并非严格圆速组合，
    // 故近地点放宽到 ±15 km 以容纳真实物理结果（函数行为正确）。
    EXPECT_NEAR(el.apoapsis, 400000.0, 5000.0);
    EXPECT_NEAR(el.periapsis, 400000.0, 15000.0);
}

// 圆轨道（用解析圆速 v=sqrt(mu/r) 反推，应得到 e 严格为 0、Ap=Pe=400km）
DS_TEST(Orbital_Circular_DerivedVelocity) {
    using DeepSpace::Planet;
    using DeepSpace::Atmosphere;
    using DeepSpace::OrbitalMechanics;
    using DeepSpace::Vec3d;
    using namespace DeepSpace;

    Planet earth("Earth", 5.9722e24, 6371000.0, Atmosphere(101325, 8500));
    const double mu = Constants::G * earth.GetMass();
    const double r = 6.771e6;
    const double vCirc = std::sqrt(mu / r);  // ≈ 7672.62 m/s

    Vec3d pos(0.0, r, 0.0);
    Vec3d vel(vCirc, 0.0, 0.0);

    auto el = OrbitalMechanics::CalculateElements(pos, vel, earth);
    EXPECT_NEAR(el.eccentricity, 0.0, 1e-6);
    EXPECT_TRUE(el.isBound);
    EXPECT_NEAR(el.apoapsis, 400000.0, 1.0);
    EXPECT_NEAR(el.periapsis, 400000.0, 1.0);
}

// 椭圆轨道：近地点 6771 km、远地点 7171 km（400 km / 800 km 高度）
// 手算：a=(rp+ra)/2=6971 km，e=(ra-rp)/(ra+rp)=0.02869，Ap=800km，Pe=400km
DS_TEST(Orbital_Elliptical_HandComputed) {
    using DeepSpace::Planet;
    using DeepSpace::Atmosphere;
    using DeepSpace::OrbitalMechanics;
    using DeepSpace::Vec3d;
    using namespace DeepSpace;

    Planet earth("Earth", 5.9722e24, 6371000.0, Atmosphere(101325, 8500));
    const double mu = Constants::G * earth.GetMass();
    const double rp = 6.771e6;
    const double ra = 7.171e6;
    const double a = (rp + ra) / 2.0;
    const double vp = std::sqrt(mu * (2.0 / rp - 1.0 / a));  // 近地点切向速度

    Vec3d pos(rp, 0.0, 0.0);
    Vec3d vel(0.0, vp, 0.0);

    auto el = OrbitalMechanics::CalculateElements(pos, vel, earth);
    EXPECT_TRUE(el.isBound);
    EXPECT_NEAR(el.semiMajorAxis, 6971000.0, 1.0);
    EXPECT_NEAR(el.eccentricity, 0.0286902883, 1e-6);
    EXPECT_NEAR(el.apoapsis, 800000.0, 1.0);
    EXPECT_NEAR(el.periapsis, 400000.0, 1.0);
}

// ---------------------------------------------------------------------------
// 2) 推进 EnginePart 一致性 + O/F 质量比
// ---------------------------------------------------------------------------
DS_TEST(Engine_OxFuelFractionSumIsOne) {
    using DeepSpace::EnginePart;
    using DeepSpace::PropellantType;

    // LH2/LOX 双组元，混合比 O/F = 6.0
    EnginePart engine("E", 1000.0, 981000.0, 300.0, 340.0,
                      PropellantType::LH2, PropellantType::LOX, 6.0);

    const double sum = engine.GetFuelMassFraction() + engine.GetOxidizerMassFraction();
    EXPECT_NEAR(sum, 1.0, 1e-9);
    // 各分量本身合理：燃料 1/7、氧化剂 6/7
    EXPECT_NEAR(engine.GetFuelMassFraction(), 1.0 / 7.0, 1e-12);
    EXPECT_NEAR(engine.GetOxidizerMassFraction(), 6.0 / 7.0, 1e-12);
}

DS_TEST(Engine_ThrustVacVsSL) {
    using DeepSpace::EnginePart;
    using DeepSpace::PropellantType;
    using namespace DeepSpace;

    const double maxThrustSL = 981000.0;  // N
    const double ispSL = 300.0;           // s
    const double ispVac = 340.0;          // s
    const double g0 = Constants::g0;      // 9.80665

    EnginePart engine("E", 1000.0, maxThrustSL, ispSL, ispVac,
                      PropellantType::LH2, PropellantType::LOX, 6.0);
    engine.SetActive(true);
    engine.SetThrottle(1.0);  // 满节流，才能与公式 mdot*g0*isp 对齐

    const double mDot = engine.GetMaxMassFlowRate();              // = maxThrustSL/(ispSL*g0)
    const double expectedVac = mDot * g0 * ispVac;               // 真空推力
    const double expectedSL = mDot * g0 * ispSL;                 // 海平推力

    // 真空：ambientPressure = 0
    EXPECT_NEAR(engine.GetThrust(0.0), expectedVac, 1e-6);
    // 海平：ambientPressure = 101325
    EXPECT_NEAR(engine.GetThrust(101325.0), expectedSL, 1e-6);
    // 真空推力应大于海平推力
    EXPECT_GT(engine.GetThrust(0.0), engine.GetThrust(101325.0));
}

DS_TEST(Engine_MassFlowRateThrottle) {
    using DeepSpace::EnginePart;
    using DeepSpace::PropellantType;

    const double maxThrustSL = 981000.0;
    const double ispSL = 300.0;

    EnginePart engine("E", 1000.0, maxThrustSL, ispSL, 340.0,
                      PropellantType::LH2, PropellantType::LOX, 6.0);
    engine.SetActive(true);

    const double mDot = engine.GetMaxMassFlowRate();

    engine.SetThrottle(1.0);
    EXPECT_NEAR(engine.GetCurrentMassFlowRate(), mDot, 1e-9);

    engine.SetThrottle(0.5);
    EXPECT_NEAR(engine.GetCurrentMassFlowRate(), 0.5 * mDot, 1e-9);

    // 关闭（未激活）应无流量、无推力
    engine.SetActive(false);
    EXPECT_NEAR(engine.GetCurrentMassFlowRate(), 0.0, 1e-12);
    EXPECT_NEAR(engine.GetThrust(0.0), 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// 3) 气动阻力系数 Aerodynamics::GetDragCoefficient(mach)
// ---------------------------------------------------------------------------
DS_TEST(Aero_DragCoefficient_Regimes) {
    using DeepSpace::Aerodynamics;

    // 亚音速：mach < 0.8 -> 0.3
    EXPECT_NEAR(Aerodynamics::GetDragCoefficient(0.0), 0.3, 1e-9);
    EXPECT_NEAR(Aerodynamics::GetDragCoefficient(0.5), 0.3, 1e-9);
    EXPECT_NEAR(Aerodynamics::GetDragCoefficient(0.79), 0.3, 1e-9);

    // 跨音速尖峰：0.8 < mach < 1.2 -> 明显大于 0.3（mach=1.0 处峰值 0.8）
    EXPECT_GT(Aerodynamics::GetDragCoefficient(1.0), 0.3);
    EXPECT_NEAR(Aerodynamics::GetDragCoefficient(1.0), 0.8, 1e-9);
    EXPECT_GT(Aerodynamics::GetDragCoefficient(0.9), 0.3);
    EXPECT_GT(Aerodynamics::GetDragCoefficient(1.1), 0.3);

    // 超音速：mach > 1.2 递减但仍 > 0.3
    const double cd12 = Aerodynamics::GetDragCoefficient(1.2);
    const double cd20 = Aerodynamics::GetDragCoefficient(2.0);
    const double cd50 = Aerodynamics::GetDragCoefficient(5.0);
    EXPECT_GT(cd12, 0.3);
    EXPECT_GT(cd20, 0.3);
    EXPECT_GT(cd50, 0.3);
    EXPECT_LT(cd20, cd12);  // 递减
    EXPECT_LT(cd50, cd20);
}

// ---------------------------------------------------------------------------
// 4) 物理积分 PhysicsBody::Update（单一恒定力，半隐式欧拉）
// ---------------------------------------------------------------------------
DS_TEST(PhysicsBody_ConstantForceIntegration) {
    using DeepSpace::PhysicsBody;
    using DeepSpace::Vec3d;

    PhysicsBody body;
    body.SetMass(1000.0);
    body.SetPosition(Vec3d(0.0, 0.0, 0.0));
    body.SetVelocity(Vec3d(10.0, 0.0, 0.0));

    const double dt = 0.5;
    const Vec3d force(0.0, 0.0, 100.0);  // 恒定力 100 N
    const Vec3d a = force / 1000.0;       // 加速度 = 0.1 m/s^2 (z 方向)

    body.AddForce(force);
    body.Update(dt);

    // v = v0 + a*dt = (10, 0, 0) + (0,0,0.1)*0.5 = (10,0,0.05)
    EXPECT_NEAR(body.GetVelocity().x, 10.0, 1e-12);
    EXPECT_NEAR(body.GetVelocity().z, 0.05, 1e-12);
    // 半隐式欧拉：pos = pos0 + v_new*dt = (0,0,0) + (0,0,0.05)*0.5 = (0,0,0.025)
    EXPECT_NEAR(body.GetPosition().z, 0.025, 1e-12);

    // 力在 Update 后应被清零
    EXPECT_NEAR(body.GetAccumulatedForce().Length(), 0.0, 1e-12);
}

DS_TEST(PhysicsBody_MultiStepIntegration) {
    using DeepSpace::PhysicsBody;
    using DeepSpace::Vec3d;

    PhysicsBody body;
    body.SetMass(10.0);
    body.SetPosition(Vec3d(0.0, 0.0, 0.0));
    body.SetVelocity(Vec3d(0.0, 0.0, 0.0));

    const double dt = 1.0;
    const Vec3d force(20.0, 0.0, 0.0);  // a = 2 m/s^2
    const double a = 2.0;

    // 两步
    body.AddForce(force); body.Update(dt);
    body.AddForce(force); body.Update(dt);

    // 半隐式欧拉两步：
    // step1: v=2, pos=2
    // step2: v=4, pos=2+4=6
    EXPECT_NEAR(body.GetVelocity().x, 4.0, 1e-12);
    EXPECT_NEAR(body.GetPosition().x, 6.0, 1e-12);
}

// ===========================================================================
// 测试入口
// ===========================================================================
int main() {
    for (auto& tc : ds_test::registry()) {
        try {
            tc.fn();
        } catch (const std::exception& ex) {
            ++ds_test::g_failed;
            std::cout << "[ EXCEPTION ] " << tc.name << " :: " << ex.what() << "\n";
        } catch (...) {
            ++ds_test::g_failed;
            std::cout << "[ EXCEPTION ] " << tc.name << " :: unknown\n";
        }
    }
    std::cout << "\n========================================\n";
    std::cout << "Passed: " << ds_test::g_passed
              << "   Failed: " << ds_test::g_failed << "\n";
    std::cout << "========================================\n";
    return ds_test::g_failed == 0 ? 0 : 1;
}
