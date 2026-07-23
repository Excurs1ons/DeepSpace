#pragma once
// ============================================================
// LambertSolver.h — Lambert 转移求解器 + Hohmann 转移 + 交会 Δv
// ------------------------------------------------------------
// 接口契约（跨代理调用，签名不可改）：
//   namespace DeepSpace {
//     struct LambertSolution { Vec3d departureVelocity; Vec3d arrivalVelocity;
//                              double transferTime; bool valid; };
//     struct HohmannTransfer { double deltaV1; double deltaV2; double totalDeltaV;
//                              double transferTime; bool valid; };
//     class LambertSolver {
//       static LambertSolution Solve(const Vec3d& r1, const Vec3d& r2,
//                                    double tofSeconds, double mu,
//                                    bool shortWay = true);
//       static HohmannTransfer Hohmann(double r1, double r2, double mu);
//       static double RendezvousDeltaV(const Vec3d& vArrival, const Vec3d& vTarget);
//     };
//   }
//
// 算法来源：
//   * Lambert 问题 —— 移植 ESA/ACT（Advanced Concepts Team）的
//     Izzo 算法（pagmo / AstroToolbox::LambertI）：以 X=log(1+cos α/2)
//     做变量变换，用 regula-falsi 求解 Lambert 飞行时间方程；再在转移
//     轨道平面内以切向/径向速度分量合成两端速度矢量。该算法在转移角
//     0/π 附近均数值稳定（参考 Izzo 2014《Revisiting Lambert's Problem》）。
//   * Hohmann 转移 —— 标准两冲量共面圆轨道转移公式
//     （Sutton & Biblarz《Rocket Propulsion Elements》）。
//   * 交会 Δv —— 到达速度与目标轨道速度之差的模。
//
// 转移角 0 与 π 为奇点（轨道面不定）：本实现对 r2 做微小旋转以打破
// 退化，给出确定且物理自洽的解（平面选择为约定值，已注释说明）。
// 所有标量计算使用 double；针对除零 / 退化情形返回 valid=false。
// ============================================================

#include <cmath>
#include <algorithm>
#include <limits>

#include "../environment/Planet.h"   // 提供 DeepSpace::Vec3d 别名
#include "../core/Constants.h"        // 提供 DeepSpace::Constants::G

// 确保 M_PI 可用（严格 -std=c++20 下 <cmath> 可能不定义）
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

namespace DeepSpace {

    // ---------------- 结果结构体（接口契约） ----------------
    struct LambertSolution {
        Vec3d departureVelocity{};   // 出发端速度 (m/s)
        Vec3d arrivalVelocity{};     // 到达端速度 (m/s)
        double transferTime = 0.0;   // 飞行时间 (s)
        bool valid = false;          // 求解是否成功
    };

    struct HohmannTransfer {
        double deltaV1 = 0.0;       // 第一冲量 Δv (m/s)
        double deltaV2 = 0.0;       // 第二冲量 Δv (m/s)
        double totalDeltaV = 0.0;    // 两冲量之和 (m/s)
        double transferTime = 0.0;   // 半椭圆转移时间 (s)
        bool valid = false;          // 计算是否有效
    };

    // 数值钳制辅助
    namespace detail {
        inline double Clamp(double v, double lo, double hi) {
            return std::max(lo, std::min(hi, v));
        }
    }

    class LambertSolver {
    public:
        // ============================================================
        // Solve —— Izzo/ESA Lambert 算法（X=log(1+cos α/2) + regula-falsi）
        //   已知：r1, r2 位置矢量；tofSeconds 飞行时间；mu 引力参数；
        //         shortWay=true 短弧（转移角 < π, 顺行），否则长弧。
        //   返回：两端速度矢量；无效输入 / 不收敛返回 valid=false。
        // ============================================================
        static LambertSolution Solve(const Vec3d& r1, const Vec3d& r2,
                                     double tofSeconds, double mu,
                                     bool shortWay = true) {
            LambertSolution result;
            result.transferTime = tofSeconds;

            // ---- 数值保护：参数与几何退化检查 ----
            if (mu <= 0.0 || tofSeconds <= 0.0) return result;
            const double R = r1.Length();           // r1 模长（归一化基准）
            if (R <= 0.0) return result;
            const double r2mag = r2.Length();
            if (r2mag <= 0.0) return result;

            const Vec3d r1u = r1 / R;              // r1 单位矢量
            const Vec3d r2u = r2 / r2mag;          // r2 单位矢量
            const double r2mod = r2mag / R;        // 无量纲 r2 模长

            const double V = std::sqrt(mu / R);    // 特征速度
            const double T = R / V;                // 特征时间
            const double t = tofSeconds / T;       // 无量纲飞行时间

            // ---- 转移角，并处理 0/π 奇点 ----
            const double cosThetaRaw = detail::Clamp(Vec3d::Dot(r1u, r2u), -1.0, 1.0);
            const double thetaRaw = std::acos(cosThetaRaw);
            const double eps = 1e-4;
            Vec3d r2eff = r2u;
            double theta = thetaRaw;
            bool nearZero = (thetaRaw < eps);
            bool nearPi = (std::abs(thetaRaw - M_PI) < eps);
            if (nearZero) {
                // 与 r1 几乎重合：对 r2 做微小旋转以定义转移平面
                Vec3d ref(0.0, 0.0, 1.0);
                if (std::abs(Vec3d::Dot(ref, r1u)) > 0.9) ref = Vec3d(1.0, 0.0, 0.0);
                const Vec3d axis = Vec3d::Cross(r1u, ref).Normalized();
                const double cs = std::cos(eps), sn = std::sin(eps);
                r2eff = r2u * cs + Vec3d::Cross(axis, r2u) * sn
                        + axis * (Vec3d::Dot(axis, r2u) * (1.0 - cs));
                theta = std::acos(detail::Clamp(Vec3d::Dot(r1u, r2eff), -1.0, 1.0));
            } else if (!nearPi) {
                theta = thetaRaw;
                if (!shortWay) theta = 2.0 * M_PI - theta; // 长弧
            }
            // nearPi：保持 r2 不变，theta=π，平面法向在下方单独给定

            const int lw = shortWay ? 0 : 1;

            const double c = std::sqrt(1.0 + r2mod * (r2mod - 2.0 * std::cos(theta)));
            const double s = (1.0 + r2mod + c) / 2.0;
            const double am = s / 2.0;   // 最小能量椭圆半长轴

            // 无量纲飞行时间方程（Curtis / Battin 形式）
            auto x2tof = [&](double x) -> double {
                const double a = am / (1.0 - x * x);
                double alfa, beta;
                if (x < 1.0) { // 椭圆
                    beta = 2.0 * std::asin(std::sqrt(detail::Clamp((s - c) / (2.0 * a), 0.0, 1.0)));
                    if (lw) beta = -beta;
                    alfa = 2.0 * std::acos(detail::Clamp(x, -1.0, 1.0));
                } else {        // 双曲
                    alfa = 2.0 * std::acosh(x);
                    beta = 2.0 * std::asinh(std::sqrt(detail::Clamp((s - c) / (-2.0 * a), 0.0, 1.0)));
                    if (lw) beta = -beta;
                }
                if (a > 0.0) {
                    return a * std::sqrt(a) * ((alfa - std::sin(alfa)) - (beta - std::sin(beta)));
                } else {
                    return -a * std::sqrt(-a) * ((std::sinh(alfa) - alfa) - (std::sinh(beta) - beta));
                }
            };

            // Regula-falsi 在 log(x2tof) 上求根（log 变换使曲线近直线、保证收敛）
            double x1 = std::log(0.4767), x2 = std::log(1.5233);
            double y1 = std::log(x2tof(-0.5233)) - std::log(t);
            double y2 = std::log(x2tof(0.5233)) - std::log(t);
            double xNew = 0.0, err = 1.0;
            int iter = 0;
            const double tol = 1e-11;
            while ((err > tol) && (y1 != y2) && (iter < 100)) {
                xNew = (x1 * y2 - y1 * x2) / (y2 - y1);
                const double yNew = std::log(x2tof(std::exp(xNew) - 1.0)) - std::log(t);
                x1 = x2; y1 = y2; x2 = xNew; y2 = yNew;
                err = std::fabs(x1 - xNew);
                ++iter;
            }
            if (iter >= 100 || !std::isfinite(xNew)) return result; // 不收敛
            const double x = std::exp(xNew) - 1.0;

            const double a = am / (1.0 - x * x);
            double alfa, beta, eta2, eta;
            if (x < 1.0) { // 椭圆分支
                beta = 2.0 * std::asin(std::sqrt(detail::Clamp((s - c) / (2.0 * a), 0.0, 1.0)));
                if (lw) beta = -beta;
                alfa = 2.0 * std::acos(detail::Clamp(x, -1.0, 1.0));
                const double psi = (alfa - beta) / 2.0;
                eta2 = 2.0 * a * std::pow(std::sin(psi), 2.0) / s;
                eta = std::sqrt(eta2);
            } else {        // 双曲分支
                beta = 2.0 * std::asinh(std::sqrt(detail::Clamp((c - s) / (2.0 * a), 0.0, 1.0)));
                if (lw) beta = -beta;
                alfa = 2.0 * std::acosh(x);
                const double psi = (alfa - beta) / 2.0;
                eta2 = -2.0 * a * std::pow(std::sinh(psi), 2.0) / s;
                eta = std::sqrt(eta2);
            }
            if (!std::isfinite(eta) || eta <= 0.0) return result;

            const double p = (r2mod / (am * eta2)) * std::pow(std::sin(theta / 2.0), 2.0);
            const double lambda = std::sqrt(r2mod) * std::cos(theta / 2.0) / s;
            const double sigma1 = (1.0 / (eta * std::sqrt(am)))
                                  * (2.0 * lambda * am - (lambda + x * eta));

            // 转移轨道面法向（短弧取 r1 × r2，长弧取反）
            Vec3d ih = Vec3d::Cross(r1u, r2eff);
            if (ih.Length() <= 1e-12 || nearPi) {
                // 近 0/π：r1 与 r2 平行，叉积退化；选与 r1 垂直的约定法向
                Vec3d ref(0.0, 0.0, 1.0);
                if (std::abs(Vec3d::Dot(ref, r1u)) > 0.9) ref = Vec3d(1.0, 0.0, 0.0);
                ih = Vec3d::Cross(r1u, ref);
            }
            ih = ih.Normalized();
            if (lw) ih = ih * (-1.0);
            if (!std::isfinite(ih.x) || ih.Length() <= 0.0) return result;

            // 在转移平面内由径向/切向分量合成速度（无量纲）
            const double vr1 = sigma1;
            const double vt1 = std::sqrt(p);
            const Vec3d v1 = r1u * vr1 + Vec3d::Cross(ih, r1u) * vt1;

            const double vt2 = vt1 / r2mod;
            const double vr2 = -vr1 + (vt1 - vt2) / std::tan(theta / 2.0);
            const Vec3d v2 = r2eff * vr2 + Vec3d::Cross(ih, r2eff) * vt2;

            // 还原真实单位
            result.departureVelocity = v1 * V;
            result.arrivalVelocity = v2 * V;
            result.valid = true;
            return result;
        }

        // ============================================================
        // Hohmann —— 共面圆轨道两冲量转移 r1 -> r2 (半径)
        //   Δv1 = sqrt(mu/r1) * (sqrt(2 r2/(r1+r2)) - 1)
        //   Δv2 = sqrt(mu/r2) * (1 - sqrt(2 r1/(r1+r2)))
        //   t   = π sqrt( ((r1+r2)/2)^3 / mu )
        // ============================================================
        static HohmannTransfer Hohmann(double r1, double r2, double mu) {
            HohmannTransfer result;
            if (mu <= 0.0 || r1 <= 0.0 || r2 <= 0.0) {
                return result; // valid=false
            }

            const double sum = r1 + r2;
            const double term = std::sqrt(2.0 * r2 / sum);   // sqrt(2 r2/(r1+r2))
            const double termInv = std::sqrt(2.0 * r1 / sum);

            const double v1c = std::sqrt(mu / r1); // 圆轨道速度 r1
            const double v2c = std::sqrt(mu / r2); // 圆轨道速度 r2

            result.deltaV1 = v1c * (term - 1.0);
            result.deltaV2 = v2c * (1.0 - termInv);
            result.totalDeltaV = result.deltaV1 + result.deltaV2;

            const double aTrans = sum * 0.5; // 转移椭圆半长轴
            result.transferTime = M_PI * std::sqrt((aTrans * aTrans * aTrans) / mu);

            result.valid = true;
            return result;
        }

        // ============================================================
        // RendezvousDeltaV —— 交会速度增量：|vArrival - vTarget|
        // ============================================================
        static double RendezvousDeltaV(const Vec3d& vArrival, const Vec3d& vTarget) {
            return (vArrival - vTarget).Length();
        }
    };

} // namespace DeepSpace
