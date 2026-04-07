#pragma once
#include "../engine/MockEngine.h"

namespace DeepSpace {
using Vec3d = Mock::Vec3d;

struct ImpactEvent {
    Vec3d Position;
    Vec3d Velocity;
    double DiameterMM = 0.0;
    double EnergyJoules = 0.0;
};

class MicrometeoriteImpact {
public:
    MicrometeoriteImpact();

    // Calculate impact probability per time step based on NASA ORDEM2000 flux model
    double CalculateImpactProbability(double altitude, double area);

    // Generate a random impact event using Poisson distribution
    std::optional<ImpactEvent> GenerateImpact(double altitude, double area);

    // Apply impact damage to vessel subsystems
    void ApplyImpactToVessel(ImpactEvent& impact, double& tpsDamage, double& structuralDamage,
                             double& propulsionDamage, double& lifeSupportDamage);

    // Main update loop - check for and process impacts
    void Update(double dt, double altitude, double area, double& tpsDamage, double& structuralDamage,
                double& propulsionDamage, double& lifeSupportDamage);

    // Get statistics
    int GetImpactCount() const { return m_ImpactCount; }
    double GetTotalEnergy() const { return m_TotalEnergy; }

private:
    // Generate random impact location on vessel surface (hemisphere distribution)
    Vec3d GenerateSurfaceLocation();

    // Generate random meteorite diameter based on flux model
    double GenerateMeteoriteDiameter();

    // Calculate crater diameter from impact energy
    double CalculateCraterDiameter(double energyJoules);

    // Poisson random number generator
    int PoissonRandom(double lambda);

    int m_ImpactCount = 0;
    double m_TotalEnergy = 0.0;
    double m_RandomSeed = 12345.0;

    // NASA ORDEM2000 inspired constants
    static constexpr double FLUX_SCALE = 1e-7;
    static constexpr double SCALE_HEIGHT = 700000.0;  // 700 km - characteristic height for LEO
    static constexpr double TYPICAL_VELOCITY = 20000.0;  // 20 km/s
    static constexpr double CRATER_COEFFICIENT = 0.3;
    static constexpr double CRATER_EXPONENT = 1.0 / 3.0;
};

}