#pragma once
#include <PrismaEngine.h>
#include <cmath>
#include "../core/Constants.h"
#include "../core/Vec3d.h"

namespace DeepSpace {

    class Atmosphere {
    public:
        Atmosphere(double seaLevelPressure, double scaleHeight)
            : m_SeaLevelPressure(seaLevelPressure), m_ScaleHeight(scaleHeight) {}

        double GetPressure(double altitude) const {
            if (altitude < 0) return m_SeaLevelPressure;
            if (altitude > m_ScaleHeight * 10) return 0.0;
            return m_SeaLevelPressure * std::exp(-altitude / m_ScaleHeight);
        }

        double GetDensity(double altitude) const {
            const double p = GetPressure(altitude);
            const double rho_sl = 1.225;
            return rho_sl * (p / m_SeaLevelPressure);
        }

    private:
        double m_SeaLevelPressure;
        double m_ScaleHeight;
    };

    class Planet {
    public:
        Planet(const std::string& name, double mass, double radius, const Atmosphere& atmosphere)
            : m_Name(name), m_Mass(mass), m_Radius(radius), m_Atmosphere(atmosphere) {}

        Prisma::Vec3d GetGravityAt(const Prisma::Vec3d& position) const {
            const double r = position.Length();
            if (r == 0.0) return {0.0, 0.0, 0.0};

            const double gMag = (Constants::G * m_Mass) / (r * r);
            return position.Normalized() * -gMag;
        }

        double GetAltitude(const Prisma::Vec3d& position) const {
            return position.Length() - m_Radius;
        }

        const Atmosphere& GetAtmosphere() const { return m_Atmosphere; }
        double GetRadius() const { return m_Radius; }
        double GetMass() const { return m_Mass; }

    private:
        std::string m_Name;
        double m_Mass;
        double m_Radius;
        Atmosphere m_Atmosphere;
    };
}
