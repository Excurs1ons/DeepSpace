#pragma once
#include <cmath>
#include "../engine/MockEngine.h"

namespace DeepSpace {
using Vec3d = Mock::Vec3d;

    class Atmosphere {
    public:
        Atmosphere(double seaLevelPressure, double scaleHeight)
            : m_SeaLevelPressure(seaLevelPressure), m_ScaleHeight(scaleHeight) {}

        double GetPressure(double altitude) const {
            const double h = std::max(0.0, altitude);
            const LayerState state = EvaluateISA(h);
            return std::max(0.0, state.pressurePa);
        }

        double GetDensity(double altitude) const {
            const double h = std::max(0.0, altitude);
            const LayerState state = EvaluateISA(h);
            if (state.temperatureK <= 0.0) return 0.0;
            return std::max(0.0, state.pressurePa / (kAirGasConstant * state.temperatureK));
        }

        double GetTemperature(double altitude) const {
            const double h = std::max(0.0, altitude);
            return EvaluateISA(h).temperatureK;
        }

        double GetSpeedOfSound(double altitude) const {
            const double temperatureK = GetTemperature(altitude);
            if (temperatureK <= 0.0) return 0.0;
            return std::sqrt(kGammaAir * kAirGasConstant * temperatureK);
        }

    private:
        struct LayerState {
            double pressurePa = 0.0;
            double temperatureK = 0.0;
        };

        // US Standard Atmosphere 1976 (troposphere to mesosphere, up to 84.852 km).
        LayerState EvaluateISA(double altitude) const {
            constexpr double g0 = 9.80665;
            constexpr double R = 8.3144598;
            constexpr double M = 0.0289644;
            constexpr double exponentScale = (g0 * M) / R;

            struct Layer {
                double hBase;
                double hTop;
                double tBase;
                double pBase;
                double lapse;
            };

            constexpr Layer layers[] = {
                {0.0,     11000.0, 288.15, 101325.0,  -0.0065},
                {11000.0, 20000.0, 216.65, 22632.06,   0.0},
                {20000.0, 32000.0, 216.65, 5474.889,   0.001},
                {32000.0, 47000.0, 228.65, 868.0187,   0.0028},
                {47000.0, 51000.0, 270.65, 110.9063,   0.0},
                {51000.0, 71000.0, 270.65, 66.93887,  -0.0028},
                {71000.0, 84852.0, 214.65, 3.956420,  -0.002}
            };

            for (const Layer& layer : layers) {
                if (altitude <= layer.hTop) {
                    const double dh = altitude - layer.hBase;
                    if (std::abs(layer.lapse) < 1e-12) {
                        const double pressure = layer.pBase * std::exp(-(exponentScale * dh) / layer.tBase);
                        return {pressure, layer.tBase};
                    }

                    const double temperature = layer.tBase + layer.lapse * dh;
                    const double pressure = layer.pBase * std::pow(layer.tBase / temperature, exponentScale / layer.lapse);
                    return {pressure, temperature};
                }
            }

            // Exosphere-facing extension: continue from ISA top boundary with simple isothermal decay.
            constexpr double hTop = 84852.0;
            constexpr double tTop = 186.946;
            constexpr double pTop = 0.3734;
            const double dh = altitude - hTop;
            const double pressure = pTop * std::exp(-(exponentScale * dh) / tTop);
            return {pressure, tTop};
        }

        static constexpr double kAirGasConstant = 287.05287;
        static constexpr double kGammaAir = 1.4;
        double m_SeaLevelPressure;
        double m_ScaleHeight;
    };

    class Planet {
    public:
        Planet(const std::string& name, double mass, double radius, const Atmosphere& atmosphere)
            : m_Name(name), m_Mass(mass), m_Radius(radius), m_Atmosphere(atmosphere) {}

        Vec3d GetGravityAt(const Vec3d& position) const {
            const double r = position.Length();
            if (r == 0.0) return {0.0, 0.0, 0.0};

            const double gMag = (6.67430e-11 * m_Mass) / (r * r);
            return position.Normalized() * -gMag;
        }

        double GetAltitude(const Vec3d& position) const {
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
