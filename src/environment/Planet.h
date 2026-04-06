#pragma once
#include <PrismaEngine.h>
#include <cmath>
#include "../core/Constants.h"

namespace DeepSpace {

    class Atmosphere {
    public:
        Atmosphere(double seaLevelPressure, double scaleHeight)
            : m_SeaLevelPressure(seaLevelPressure), m_ScaleHeight(scaleHeight) {}

        // Returns pressure in Pascals (approx 101325 at sea level for Earth)
        double GetPressure(double altitude) const {
            if (altitude < 0) return m_SeaLevelPressure;
            if (altitude > m_ScaleHeight * 10) return 0.0;
            return m_SeaLevelPressure * std::exp(-altitude / m_ScaleHeight);
        }

        // Returns density in kg/m^3
        double GetDensity(double altitude) const {
            // rho = p / (R_specific * T)
            // For a simplified isothermal model:
            double p = GetPressure(altitude);
            double rho_sl = 1.225; // kg/m^3 at sea level
            return rho_sl * (p / m_SeaLevelPressure);
        }

    private:
        double m_SeaLevelPressure; // Pascals
        double m_ScaleHeight; // meters
    };

    class Planet {
    public:
        Planet(const std::string& name, double mass, double radius, const Atmosphere& atmosphere)
            : m_Name(name), m_Mass(mass), m_Radius(radius), m_Atmosphere(atmosphere) {}

        Prisma::Vec3d GetGravityAt(const Prisma::Vec3d& position) const {
            double r = position.Length();
            if (r == 0) return {0, 0, 0};
            
            // F/m = -G * M / r^2
            double g_mag = (Constants::G * m_Mass) / (r * r);
            return position.Normalized() * -g_mag;
        }

        double GetAltitude(const Prisma::Vec3d& position) const {
            return position.Length() - m_Radius;
        }

        const Atmosphere& GetAtmosphere() const { return m_Atmosphere; }
        double GetRadius() const { return m_Radius; }

    private:
        std::string m_Name;
        double m_Mass;
        double m_Radius;
        Atmosphere m_Atmosphere;
    };

}
