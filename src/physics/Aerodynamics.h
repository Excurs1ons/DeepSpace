#pragma once
#include <algorithm>
#include <cmath>
#include "../environment/Planet.h"
#include "PhysicsBody.h"

namespace DeepSpace {

    class Aerodynamics {
    public:
        static double GetSpeedOfSound(double altitude) {
            const double base = 340.0;
            if (altitude <= 0.0) return base;
            if (altitude > 11000.0) return 295.0;
            return base - (base - 295.0) * (altitude / 11000.0);
        }

        static double GetMachNumber(double velocityMag, double speedOfSound) {
            if (speedOfSound <= 0.0) return 0.0;
            return velocityMag / speedOfSound;
        }

        static double GetDragCoefficient(double mach) {
            const double baseCd = 0.3;
            if (mach > 0.8 && mach < 1.2) {
                const double t = (mach - 0.8) / 0.4;
                return baseCd + (0.5 * std::sin(t * 3.14159265358979323846));
            }
            if (mach >= 1.2) {
                return baseCd + 0.5 * std::exp(-(mach - 1.2));
            }
            return baseCd;
        }

        static void ApplyAerodynamics(PhysicsBody& body, const Atmosphere& atmosphere, double altitude) {
            const Vec3d velocity = body.GetVelocity();
            const double speed = velocity.Length();
            if (speed < 0.1 || altitude > 100000.0) return;

            const double density = atmosphere.GetDensity(altitude);
            if (density <= 0.0001) return;

            const double sos = GetSpeedOfSound(altitude);
            const double mach = GetMachNumber(speed, sos);
            const double cd = GetDragCoefficient(mach);
            const double area = 10.7;

            const double q = 0.5 * density * speed * speed;
            const double dragMag = q * cd * area;

            const Vec3d velDir = velocity.Normalized();
            const Vec3d dragDir = velDir * -1.0;
            const Vec3d dragForce = dragDir * dragMag;

            const Vec3d orientation = body.GetOrientationVec3();
            const double dot = std::clamp(Vec3d::Dot(orientation, velDir), -1.0, 1.0);
            const double aoa = std::acos(dot);

            double cl = 0.0;
            if (aoa < 0.35) {
                cl = aoa * 2.0 * 3.14159265358979323846;
            }

            Vec3d liftDir(-dragDir.y, dragDir.x, 0.0);
            const double crossZ = orientation.x * velocity.y - orientation.y * velocity.x;
            if (crossZ > 0.0) {
                liftDir = liftDir * -1.0;
            }

            const Vec3d liftForce = liftDir * (q * cl * area * 0.1);
            body.AddForce(dragForce + liftForce);
        }
    };
}
