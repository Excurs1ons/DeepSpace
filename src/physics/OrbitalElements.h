#pragma once
#include <PrismaEngine.h>
#include "../environment/Planet.h"
#include <cmath>

namespace DeepSpace {

    struct OrbitalElements {
        double semiMajorAxis; // a
        double eccentricity;  // e
        double apoapsis;      // Ap
        double periapsis;     // Pe
        double inclination;   // i (omitted for 2D simplified math, but placeholder)
    };

    class OrbitalMechanics {
    public:
        // Calculates Keplerian elements from state vectors (position, velocity)
        static OrbitalElements CalculateElements(const Prisma::Vec3d& pos, const Prisma::Vec3d& vel, const Planet& planet) {
            double mu = Constants::G * 5.9722e24; // Standard gravitational parameter (mu = GM) for Earth
            
            double r = pos.Length();
            double v = vel.Length();

            // Specific orbital energy: E = v^2 / 2 - mu / r
            double specificEnergy = (v * v) / 2.0 - (mu / r);

            // Semi-major axis: a = -mu / (2E)
            double a = -mu / (2.0 * specificEnergy);

            // Specific angular momentum vector: h = r x v (Simplified for 2D here)
            // For a full 3D simulation, we'd use cross product. We'll approximate magnitude for now:
            // Since our rocket goes straight up, angular momentum is currently ~0, but let's provide the true math.
            Prisma::Vec3d h_vec(
                pos.y * vel.z - pos.z * vel.y,
                pos.z * vel.x - pos.x * vel.z,
                pos.x * vel.y - pos.y * vel.x
            );
            double h = h_vec.Length();

            // Eccentricity: e = sqrt(1 + (2E h^2) / mu^2)
            // Or using e_vec = (v x h)/mu - r_dir
            double e = std::sqrt(std::max(0.0, 1.0 + (2.0 * specificEnergy * h * h) / (mu * mu)));

            // Apoapsis and Periapsis (from center of body)
            double r_a = a * (1.0 + e);
            double r_p = a * (1.0 - e);

            // Convert to altitude
            double apoapsisAlt = r_a - planet.GetRadius();
            double periapsisAlt = r_p - planet.GetRadius();

            return { a, e, apoapsisAlt, periapsisAlt, 0.0 };
        }
    };
}
