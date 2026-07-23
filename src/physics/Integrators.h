#pragma once
#include <vector>
#include <functional>
#include <cmath>
#include <algorithm>
#include <cstddef>

#include "../engine/MockEngine.h"
#include "../core/Constants.h"

namespace DeepSpace {
using Vec3d = Mock::Vec3d;

// ============================================================================
// Integrators.h —— 数值积分器层 (任务4: 数值积分器升级 RK4 + 自适应)
//
// 提供三种能力:
//   1. RK4()        经典 4 阶 Runge-Kutta 定步长积分器。
//   2. AdaptiveStep()  RKF45 (Runge-Kutta-Fehlberg) 4/5 阶嵌入式积分器,
//                     利用 4 阶与 5 阶解之差做误差估计, 自适应缩放步长,
//                     在达到 tEnd 时自动停机。t 与 dt 均为输入输出。
//   3. PropagateTwoBody() 在中心引力 mu 下用 RK4 定步长递推二体轨道,
//                     状态为 6 维 [x,y,z,vx,vy,vz]。
//
// 设计约束 (跨代理契约):
//   - 全部 double 精度。
//   - 纯头文件、内联实现, 无外部依赖。
//   - 退化保护: 空状态直接返回、零/负步长与容差做合理默认与裁剪,
//     二体中心引力对 r≈0 做防除零处理。
// ============================================================================

using StateVector = std::vector<double>;
using DerivativeFunc = std::function<StateVector(double t, const StateVector& y)>;

class Integrators {
public:
    // ----------------------------------------------------------------------
    // RK4: 经典 4 阶 Runge-Kutta 定步长。
    //   y_{n+1} = y_n + dt/6 * (k1 + 2k2 + 2k3 + k4)
    // 其中:
    //   k1 = f(t,            y)
    //   k2 = f(t + dt/2,     y + dt/2 * k1)
    //   k3 = f(t + dt/2,     y + dt/2 * k2)
    //   k4 = f(t + dt,       y + dt   * k3)
    // ----------------------------------------------------------------------
    static StateVector RK4(DerivativeFunc f, double t, const StateVector& y, double dt) {
        if (y.empty() || dt == 0.0) return y;
        const size_t n = y.size();

        const StateVector k1 = f(t, y);
        if (k1.size() != n) return y;  // 退化保护: 维度不一致直接放弃

        const StateVector k2 = f(t + 0.5 * dt, AddScaled(y, k1, 0.5 * dt));
        const StateVector k3 = f(t + 0.5 * dt, AddScaled(y, k2, 0.5 * dt));
        const StateVector k4 = f(t + dt, AddScaled(y, k3, dt));

        StateVector yNext(n);
        const double sixth = dt / 6.0;
        for (size_t i = 0; i < n; ++i) {
            yNext[i] = y[i] + sixth * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
        return yNext;
    }

    // ----------------------------------------------------------------------
    // AdaptiveStep: RKF45 自适应步长单步积分 (4/5 阶嵌入式)。
    //   - 误差估计: err = max_i |y5_i - y4_i| / (tol * max(1, |y_i|, |y5_i|))
    //   - 接受判据: err <= 1。
    //   - 步长更新: h *= safety * (tol/err)^(1/5), 并 clamp 到 [0.1, 5.0]。
    //   - t / dt 为输入输出: 接受后 t 前进, dt 建议下一步(已被 tEnd 裁剪)。
    //   - 到达或越过 tEnd 时直接返回当前状态, 不做任何推进。
    //   - 退化保护: 空状态返回; 非正步长/容差给默认; 拒绝次数与最小步长上限
    //     防止无限循环(极端情况下强制接受以推进)。
    // ----------------------------------------------------------------------
    static StateVector AdaptiveStep(DerivativeFunc f, double& t, StateVector y, double& dt, double tol, double tEnd) {
        constexpr double kSafety    = 0.9;
        constexpr double kMinFactor = 0.1;
        constexpr double kMaxFactor = 5.0;
        constexpr double kMinStep   = 1e-12;  // 步长地板, 防除零/无限循环
        constexpr int    kMaxReject = 50;

        if (y.empty()) return y;
        if (dt <= 0.0) dt = 1e-3;
        if (tol <= 0.0) tol = 1e-6;
        if (t >= tEnd) return y;  // 已到达终点

        double h = dt;
        if (h > tEnd - t) h = tEnd - t;  // 不越过终点

        int reject = 0;
        while (true) {
            RKF45Result r = RKF45Step(f, t, y, h);
            if (r.y4.size() != y.size()) return y;  // 退化保护

            // 加权相对误差估计
            double err = 0.0;
            for (size_t i = 0; i < y.size(); ++i) {
                const double denom = tol * std::max({1.0, std::abs(y[i]), std::abs(r.y5[i])});
                err = std::max(err, std::abs(r.y5[i] - r.y4[i]) / denom);
            }

            if (err <= 1.0) {
                // 接受高精度的 5 阶解
                t += h;
                double factor = kSafety * std::pow(err > 0.0 ? 1.0 / err : kMaxFactor, 0.2);
                factor = std::clamp(factor, kMinFactor, kMaxFactor);
                double newDt = h * factor;
                if (t < tEnd) newDt = std::min(newDt, tEnd - t);  // 下一步不越过终点
                dt = newDt;
                return r.y5;
            }

            // 拒绝: 缩小步长后重试
            ++reject;
            double factor = kSafety * std::pow(1.0 / err, 0.2);
            factor = std::clamp(factor, kMinFactor, 0.9);
            h *= factor;

            if (reject > kMaxReject || h < kMinStep) {
                // 放弃精细控制, 强制接受以推进, 避免死循环
                t += h;
                dt = h;
                return r.y5;
            }
        }
    }

    // ----------------------------------------------------------------------
    // PropagateTwoBody: 在中心引力 mu 下用 RK4 定步长递推二体轨道。
    //   状态: [x, y, z, vx, vy, vz] (6 维)
    //   加速度: a = -mu * r / |r|^3  (r≈0 时给零加速度防除零)
    //   从 t0 积分到 tEnd, 定步长 dt (末步裁剪至 tEnd-t)。
    // ----------------------------------------------------------------------
    static StateVector PropagateTwoBody(const Vec3d& pos, const Vec3d& vel, double mu, double t0, double tEnd, double dt) {
        StateVector y = {pos.x, pos.y, pos.z, vel.x, vel.y, vel.z};

        if (tEnd <= t0) return y;       // 退化保护: 无积分区间
        if (dt <= 0.0) return y;        // 退化保护: 非法步长

        DerivativeFunc twoBody = [mu](double /*t*/, const StateVector& s) -> StateVector {
            Vec3d r(s[0], s[1], s[2]);
            Vec3d a(0.0, 0.0, 0.0);
            const double rLen2 = r.LengthSquared();
            if (rLen2 > 0.0) {
                const double rLen = std::sqrt(rLen2);
                const double inv = -mu / (rLen2 * rLen);  // -mu / r^3
                a = r * inv;
            }
            return StateVector{s[3], s[4], s[5], a.x, a.y, a.z};
        };

        double t = t0;
        while (t < tEnd) {
            const double h = std::min(dt, tEnd - t);
            y = RK4(twoBody, t, y, h);
            t += h;
        }
        return y;
    }

private:
    // y + scale * k  (向量辅助运算, 保持状态维度一致)
    static StateVector AddScaled(const StateVector& y, const StateVector& k, double scale) {
        const size_t n = y.size();
        StateVector out(n);
        for (size_t i = 0; i < n; ++i) out[i] = y[i] + scale * k[i];
        return out;
    }

    struct RKF45Result {
        StateVector y4;  // 4 阶解
        StateVector y5;  // 5 阶解 (更精确, 作为接受解)
    };

    // 单次 RKF45 阶段计算, 返回 4 阶与 5 阶两个解。
    static RKF45Result RKF45Step(DerivativeFunc f, double t, const StateVector& y, double h) {
        const size_t n = y.size();
        RKF45Result res;
        res.y4.assign(n, 0.0);
        res.y5.assign(n, 0.0);

        // Butcher 表 (Fehlberg) 系数
        const double c2 = 1.0 / 4.0,        c3 = 3.0 / 8.0,  c4 = 12.0 / 13.0, c5 = 1.0, c6 = 1.0 / 2.0;

        const StateVector k1 = f(t, y);
        if (k1.size() != n) return res;

        // k2 = f(t + c2*h, y + h*(1/4 k1))
        const StateVector k2 = f(t + c2 * h, AddScaled(y, k1, (1.0 / 4.0) * h));
        // k3 = f(t + c3*h, y + h*(3/32 k1 + 9/32 k2))
        StateVector tmp(n);
        for (size_t i = 0; i < n; ++i) tmp[i] = y[i] + h * ((3.0 / 32.0) * k1[i] + (9.0 / 32.0) * k2[i]);
        const StateVector k3 = f(t + c3 * h, tmp);
        // k4 = f(t + c4*h, y + h*(1932/2197 k1 - 7200/2197 k2 + 7296/2197 k3))
        for (size_t i = 0; i < n; ++i)
            tmp[i] = y[i] + h * ((1932.0 / 2197.0) * k1[i] + (-7200.0 / 2197.0) * k2[i] + (7296.0 / 2197.0) * k3[i]);
        const StateVector k4 = f(t + c4 * h, tmp);
        // k5 = f(t + h, y + h*(439/216 k1 - 8 k2 + 3680/513 k3 - 845/4104 k4))
        for (size_t i = 0; i < n; ++i)
            tmp[i] = y[i] + h * ((439.0 / 216.0) * k1[i] + (-8.0) * k2[i] + (3680.0 / 513.0) * k3[i] + (-845.0 / 4104.0) * k4[i]);
        const StateVector k5 = f(t + c5 * h, tmp);
        // k6 = f(t + c6*h, y + h*(-8/27 k1 + 2 k2 - 3544/2565 k3 + 1859/4104 k4 - 11/40 k5))
        for (size_t i = 0; i < n; ++i)
            tmp[i] = y[i] + h * ((-8.0 / 27.0) * k1[i] + (2.0) * k2[i] + (-3544.0 / 2565.0) * k3[i] +
                                 (1859.0 / 4104.0) * k4[i] + (-11.0 / 40.0) * k5[i]);
        const StateVector k6 = f(t + c6 * h, tmp);

        // 4 阶解权重: 25/216, 0, 1408/2565, 2197/4104, -1/5, 0
        // 5 阶解权重: 16/135, 0, 6656/12825, 28561/56430, -9/50, 2/55
        for (size_t i = 0; i < n; ++i) {
            res.y4[i] = y[i] + h * ((25.0 / 216.0) * k1[i] + 0.0 * k2[i] + (1408.0 / 2565.0) * k3[i] +
                                    (2197.0 / 4104.0) * k4[i] + (-1.0 / 5.0) * k5[i] + 0.0 * k6[i]);
            res.y5[i] = y[i] + h * ((16.0 / 135.0) * k1[i] + 0.0 * k2[i] + (6656.0 / 12825.0) * k3[i] +
                                    (28561.0 / 56430.0) * k4[i] + (-9.0 / 50.0) * k5[i] + (2.0 / 55.0) * k6[i]);
        }
        return res;
    }
};

}  // namespace DeepSpace
