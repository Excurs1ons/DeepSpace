#pragma once
#include <PrismaEngine.h>
#include "../environment/Planet.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace DeepSpace {

    struct OrbitalElements {
        double semiMajorAxis; // a
        double eccentricity;  // e
        double apoapsis;      // Ap (altitude)
        double periapsis;     // Pe (altitude)
        double inclination;   // i
        bool isBound;
    };

    struct OrbitPrediction {
        double apoapsis;
        double periapsis;
        int samples;
    };

    class OrbitalMechanics {
    public:
        static OrbitalElements CalculateElements(const Prisma::Vec3d& pos, const Prisma::Vec3d& vel, const Planet& planet) {
            const double mu = Constants::G * planet.GetMass();
            const double r = pos.Length();
            if (r <= 1.0 || mu <= 0.0) {
                return {0.0, 0.0, 0.0, 0.0, 0.0, false};
            }

            const double v2 = vel.LengthSquared();
            const double specificEnergy = (v2 * 0.5) - (mu / r);

            const Prisma::Vec3d hVec = Prisma::Vec3d::Cross(pos, vel);
            const double h = hVec.Length();

            Prisma::Vec3d eVec = Prisma::Vec3d::Cross(vel, hVec) / mu;
            eVec -= pos.Normalized();
            const double e = eVec.Length();

            const bool isBound = specificEnergy < 0.0 && e < 1.0;
            const double inclination = (h > 1e-9)
                ? std::acos(std::max(-1.0, std::min(1.0, hVec.z / h)))
                : 0.0;

            if (!isBound) {
                const double periR = (e > 1e-9) ? ((h * h) / (mu * (1.0 + e))) : r;
                return {
                    std::numeric_limits<double>::infinity(),
                    e,
                    std::numeric_limits<double>::infinity(),
                    periR - planet.GetRadius(),
                    inclination,
                    false
                };
            }

            const double a = -mu / (2.0 * specificEnergy);
            const double rA = a * (1.0 + e);
            const double rP = a * (1.0 - e);

            return {
                a,
                e,
                rA - planet.GetRadius(),
                rP - planet.GetRadius(),
                inclination,
                true
            };
        }

        static OrbitPrediction PredictVacuumExtrema(
            const Prisma::Vec3d& startPos,
            const Prisma::Vec3d& startVel,
            const Planet& planet,
            double durationSeconds,
            double dtSeconds)
        {
            if (durationSeconds <= 0.0 || dtSeconds <= 0.0) {
                return {0.0, 0.0, 0};
            }

            const double mu = Constants::G * planet.GetMass();
            Prisma::Vec3d pos = startPos;
            Prisma::Vec3d vel = startVel;

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
                const Prisma::Vec3d accel = pos.Normalized() * gMag;

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

            return {
                maxR - planet.GetRadius(),
                minR - planet.GetRadius(),
                samples
            };
        }
    };
}
