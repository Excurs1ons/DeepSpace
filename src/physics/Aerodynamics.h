#pragma once
#include <cmath>
#include <PrismaEngine.h>
#include "../environment/Planet.h"
#include "PhysicsBody.h"

namespace DeepSpace {

    class Aerodynamics {
    public:
        // Calculate the speed of sound at a given altitude
        static double GetSpeedOfSound(double altitude) {
            // Simplified: Assume constant temperature for now (roughly 340 m/s)
            // In a real model, this depends on temperature which drops with altitude
            double baseSoS = 340.0;
            if (altitude > 11000.0) {
                // Stratosphere is colder, SoS drops to ~295 m/s
                return 295.0;
            }
            // Linear interpolation up to 11km
            return baseSoS - (baseSoS - 295.0) * (altitude / 11000.0);
        }

        // Calculate Mach number
        static double GetMachNumber(double velocityMag, double speedOfSound) {
            if (speedOfSound <= 0) return 0;
            return velocityMag / speedOfSound;
        }

        // Transonic drag multiplier
        static double GetDragCoefficient(double mach) {
            double baseCd = 0.3; // Base drag coefficient for a rocket shape
            
            // Supersonic drag peak (Mach 1.0 - 1.2)
            if (mach > 0.8 && mach < 1.2) {
                // Drag rises sharply
                double t = (mach - 0.8) / 0.4;
                return baseCd + (0.5 * std::sin(t * M_PI)); 
            } else if (mach >= 1.2) {
                // Slowly drops off but remains higher than subsonic
                return baseCd + 0.5 * std::exp(-(mach - 1.2));
            }
            
            return baseCd;
        }

        // Calculate aerodynamic forces (Drag and Lift)
        static void ApplyAerodynamics(PhysicsBody& body, const Atmosphere& atmosphere, double altitude) {
            Prisma::Vec3d velocity = body.GetVelocity();
            double speed = velocity.Length();
            if (speed < 0.1 || altitude > 100000.0) return; // Negligible in space or at rest

            double density = atmosphere.GetDensity(altitude);
            if (density <= 0.0001) return;

            double sos = GetSpeedOfSound(altitude);
            double mach = GetMachNumber(speed, sos);
            double cd = GetDragCoefficient(mach);

            // Cross-sectional area (approximate for a Falcon 9, radius ~1.85m -> Area ~10.7m^2)
            double area = 10.7; 

            // Dynamic pressure (q) = 0.5 * rho * v^2
            double q = 0.5 * density * speed * speed;

            // Drag force: Fd = q * Cd * A
            double dragMag = q * cd * area;
            
            Prisma::Vec3d dragDir = velocity.Normalized() * -1.0;
            Prisma::Vec3d dragForce = dragDir * dragMag;

            // Lift force (simplified): Fl = q * Cl * A
            // Lift depends on Angle of Attack (AoA)
            Prisma::Vec3d orientation = body.GetOrientation();
            double dot = Prisma::Vec3d::Dot(orientation, velocity.Normalized());
            // Clamp dot to [-1, 1] to avoid acos domain errors
            dot = std::max(-1.0, std::min(1.0, dot));
            double aoa = std::acos(dot); // Angle in radians

            // Simple lift curve (stalls at high AoA)
            double cl = 0.0;
            if (aoa < 0.35) { // ~20 degrees
                cl = aoa * 2.0 * M_PI; // Linear lift region
            } else {
                cl = 0.0; // Stalled
            }

            // Lift direction is perpendicular to velocity and pitch axis
            // For 2D flight, lift is just perpendicular to velocity
            Prisma::Vec3d liftDir(-dragDir.y, dragDir.x, 0.0); // 90 deg rotation
            
            // Determine sign of AoA to apply lift in correct direction
            double crossZ = orientation.x * velocity.y - orientation.y * velocity.x;
            if (crossZ > 0) liftDir = liftDir * -1.0;

            Prisma::Vec3d liftForce = liftDir * (q * cl * area * 0.1); // Body lift is weak on rockets

            body.AddForce(dragForce + liftForce);
        }
    };
}
