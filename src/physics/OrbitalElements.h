#pragma once
#include "../environment/Planet.h"
#include "../core/Constants.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace DeepSpace {

    struct OrbitalElements {
        double semiMajorAxis;
        double eccentricity;
        double apoapsis;
        double periapsis;
        double inclination;
        bool isBound;
    };

    struct OrbitPrediction {
        double apoapsis;
        double periapsis;
        int samples;
    };

    class OrbitalMechanics {
    public:
        static OrbitalElements CalculateElements(const Vec3d& pos, const Vec3d& vel, const Planet& planet) {
            const double mu = Constants::G * planet.GetMass();
            const double r = pos.Length();
            if (r <= 1.0 || mu <= 0.0) {
                return {0.0, 0.0, 0.0, 0.0, 0.0, false};
            }

            const double v2 = vel.LengthSquared();
            const double specificEnergy = (v2 * 0.5) - (mu / r);

            const Vec3d hVec = Vec3d::Cross(pos, vel);
            const double h = hVec.Length();

            Vec3d eVec = Vec3d::Cross(vel, hVec) / mu;
            eVec -= pos.Normalized();
            const double e = eVec.Length();

            const bool isBound = specificEnergy < 0.0 && e < 1.0;
            const double inclination = (h > 1e-9)
                ? std::acos(std::max(-1.0, std::min(1.0, hVec.z / h)))
                : 0.0;

            if (!isBound) {
                const double periR = (e > 1e-9) ? ((h * h) / (mu * (1.0 + e))) : r;
                const double planetRadius = planet.GetRadius();
                double periapsis = periR - planetRadius;
                if (periapsis < 0) periapsis = 0.0;
                
                return {
                    std::numeric_limits<double>::infinity(),
                    e,
                    std::numeric_limits<double>::infinity(),
                    periapsis,
                    inclination,
                    false
                };
            }

const double a = -mu / (2.0 * specificEnergy);
            const double rA = a * (1.0 + e);
            const double rP = a * (1.0 - e);
            const double planetRadius = planet.GetRadius();
            
            double apoapsis = rA - planetRadius;
            double periapsis = rP - planetRadius;
            
            if (periapsis < 0) periapsis = 0.0;
            if (apoapsis < 0) apoapsis = 0.0;
            
            return {
                a,
                e,
                apoapsis,
                periapsis,
                inclination,
                true
            };
        }

        static OrbitPrediction PredictVacuumExtrema(
            const Vec3d& startPos,
            const Vec3d& startVel,
            const Planet& planet,
            double durationSeconds,
            double dtSeconds)
        {
            if (durationSeconds <= 0.0 || dtSeconds <= 0.0) {
                return {0.0, 0.0, 0};
            }

            const double mu = Constants::G * planet.GetMass();
            Vec3d pos = startPos;
            Vec3d vel = startVel;

            double minR = pos.Length();
            double maxR = minR;
            int samples = 0;

            const int steps = std::max(1, static_cast<int>(durationSeconds / dtSeconds));
            for (int i = 0; i < steps; ++i) {
                const double r = pos.Length();
                if (r <= 1.0) {
                    break;
                }

                const double gMag = -(mu / (r * r));
                const Vec3d accel = pos.Normalized() * gMag;

                vel += accel * dtSeconds;
                pos += vel * dtSeconds;

                const double newR = pos.Length();
                minR = std::min(minR, newR);
                maxR = std::max(maxR, newR);
                ++samples;

                if (newR <= planet.GetRadius()) {
                    break;
                }
            }

            const double planetRadius = planet.GetRadius();
            double maxAlt = maxR - planetRadius;
            double minAlt = minR - planetRadius;
            if (minAlt < 0) minAlt = 0.0;
            if (maxAlt < 0) maxAlt = 0.0;
            
            return {
                maxAlt,
                minAlt,
                samples
            };
        }

        static double CircularOrbitVelocity(double altitude, const Planet& planet) {
            const double mu = Constants::G * planet.GetMass();
            const double r = planet.GetRadius() + altitude;
            if (r <= 0.0) return 0.0;
            return std::sqrt(mu / r);
        }

        static double DeltaVToRaiseApoapsis(double currentApoapsis, double targetApoapsis, 
                                            double periapsis, const Planet& planet) {
            const double mu = Constants::G * planet.GetMass();
            const double r_p = planet.GetRadius() + periapsis;
            const double r_a = planet.GetRadius() + currentApoapsis;
            const double r_a_target = planet.GetRadius() + targetApoapsis;
            
            if (r_p <= 0.0 || r_a <= 0.0 || r_a_target <= r_a) return 0.0;
            
            double v_p = std::sqrt(mu * (2.0 / r_p - 1.0 / ((r_a + r_p) / 2.0)));
            double v_p_new = std::sqrt(mu * (2.0 / r_p - 1.0 / ((r_a_target + r_p) / 2.0)));
            
            return v_p_new - v_p;
        }

        static double TimeToApoapsis(const Vec3d& pos, const Vec3d& vel, const Planet& planet) {
            const double mu = Constants::G * planet.GetMass();
            const double r = pos.Length();
            const double v2 = vel.LengthSquared();
            
            if (r <= planet.GetRadius()) return 0.0;
            
            const double specificEnergy = (v2 * 0.5) - (mu / r);
            if (specificEnergy >= 0.0) return 0.0;
            
            const Vec3d hVec = Vec3d::Cross(pos, vel);
            const double h = hVec.Length();
            if (h <= 1e-9) return 0.0;
            
            const double a = -mu / (2.0 * specificEnergy);
            const double period = 2.0 * M_PI * std::sqrt(a * a * a / mu);
            
            Vec3d rVec = pos.Normalized();
            Vec3d vVec = vel.Normalized();
            double cosNu = Vec3d::Dot(rVec, vVec);
            Vec3d crossRV = Vec3d::Cross(rVec, vVec);
            double sinNu = crossRV.Length() * ((crossRV.z >= 0) ? 1.0 : -1.0);
            double nu = std::atan2(sinNu, cosNu);
            if (nu < 0.0) nu += 2.0 * M_PI;
            
            double timeToApoapsis = nu > M_PI ? (2.0 * M_PI - nu) / (2.0 * M_PI) * period : nu / (2.0 * M_PI) * period;
            return timeToApoapsis;
        }
        
        static bool IsEscapeOrbit(const Vec3d& pos, const Vec3d& vel, const Planet& planet) {
            const double mu = Constants::G * planet.GetMass();
            const double r = pos.Length();
            const double v2 = vel.LengthSquared();
            if (r <= 1.0 || mu <= 0.0) return true;
            const double vEsc = std::sqrt(2.0 * mu / r);
            return v2 >= vEsc * vEsc;
        }
        
        static double GetEscapeVelocity(const Planet& planet, double altitude) {
            const double mu = Constants::G * planet.GetMass();
            const double r = planet.GetRadius() + altitude;
            if (r <= 0.0) return 0.0;
            return std::sqrt(2.0 * mu / r);
        }
        
        static double EstimateCircularizationDV(double currentPeriapsis, double targetPeriapsis, const Planet& planet) {
            const double mu = Constants::G * planet.GetMass();
            const double r1 = planet.GetRadius() + currentPeriapsis;
            const double r2 = planet.GetRadius() + targetPeriapsis;
            
            if (r1 <= 0.0 || r2 <= 0.0) return 0.0;
            const double v1 = std::sqrt(mu / r1);
            const double v2 = std::sqrt(mu / r2);
            return std::abs(v2 - v1);
        }
        
        static double EstimateMOIDV(double currentApoapsis, double currentPeriapsis, double targetApoapsis, const Planet& planet) {
            const double mu = Constants::G * planet.GetMass();
            const double rp = planet.GetRadius() + currentPeriapsis;
            const double ra = planet.GetRadius() + currentApoapsis;
            const double raTarget = planet.GetRadius() + targetApoapsis;
            
            if (rp <= 0.0 || ra <= 0.0 || raTarget <= ra) return 0.0;
            const double vP = std::sqrt(mu * (2.0 / rp - 1.0 / ((ra + rp) / 2.0)));
            const double vPTarget = std::sqrt(mu * (2.0 / rp - 1.0 / ((raTarget + rp) / 2.0)));
            return std::abs(vPTarget - vP);
        }
    };
}
