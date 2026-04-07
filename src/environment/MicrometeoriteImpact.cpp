#include "MicrometeoriteImpact.h"
#include <random>
#include <cmath>

namespace DeepSpace {

MicrometeoriteImpact::MicrometeoriteImpact()
    : m_ImpactCount(0), m_TotalEnergy(0.0), m_RandomSeed(12345.0) {
}

double MicrometeoriteImpact::CalculateImpactProbability(double altitude, double area) {
    if (area <= 0.0 || altitude <= 0.0) {
        return 0.0;
    }

    double flux = FLUX_SCALE * std::exp(-altitude / SCALE_HEIGHT);
    double probability = flux * area;

    return probability;
}

std::optional<ImpactEvent> MicrometeoriteImpact::GenerateImpact(double altitude, double area) {
    double lambda = CalculateImpactProbability(altitude, area);

    if (lambda <= 0.0) {
        return std::nullopt;
    }

    int impactCount = PoissonRandom(lambda);

    if (impactCount <= 0) {
        return std::nullopt;
    }

    ImpactEvent event;
    event.Position = GenerateSurfaceLocation();
    event.Velocity = Vec3d(0.0, 0.0, TYPICAL_VELOCITY);
    event.DiameterMM = GenerateMeteoriteDiameter();

    double meteoriteMass = 2.5 * std::pow(event.DiameterMM, 3) * 1e-9;
    event.EnergyJoules = 0.5 * meteoriteMass * TYPICAL_VELOCITY * TYPICAL_VELOCITY;

    m_ImpactCount += impactCount;
    m_TotalEnergy += event.EnergyJoules * static_cast<double>(impactCount);

    return event;
}

void MicrometeoriteImpact::ApplyImpactToVessel(ImpactEvent& impact, double& tpsDamage,
                                                double& structuralDamage,
                                                double& propulsionDamage,
                                                double& lifeSupportDamage) {
    double craterDiameter = CalculateCraterDiameter(impact.EnergyJoules);

    double tpsContrib = craterDiameter * 0.01;
    double structuralContrib = impact.EnergyJoules * 1e-9;
    double propulsionContrib = impact.EnergyJoules * 5e-10;
    double lifeSupportContrib = impact.EnergyJoules * 2e-10;

    tpsDamage += tpsContrib;
    structuralDamage += structuralContrib;
    propulsionDamage += propulsionContrib;
    lifeSupportDamage += lifeSupportContrib;
}

void MicrometeoriteImpact::Update(double dt, double altitude, double area, double& tpsDamage,
                                 double& structuralDamage, double& propulsionDamage,
                                 double& lifeSupportDamage) {
    if (dt <= 0.0 || area <= 0.0) {
        return;
    }

    double timeScaledArea = area * dt;
    auto impactOpt = GenerateImpact(altitude, timeScaledArea);

    if (impactOpt.has_value()) {
        ApplyImpactToVessel(impactOpt.value(), tpsDamage, structuralDamage,
                           propulsionDamage, lifeSupportDamage);
    }
}

Vec3d MicrometeoriteImpact::GenerateSurfaceLocation() {
    std::mt19937 gen(static_cast<unsigned int>(m_RandomSeed++));
    std::uniform_real_distribution<double> thetaDist(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<double> phiDist(0.0, M_PI / 2.0);

    double theta = thetaDist(gen);
    double phi = phiDist(gen);

    double x = std::sin(phi) * std::cos(theta);
    double y = std::sin(phi) * std::sin(theta);
    double z = std::cos(phi);

    return Vec3d(x, y, z);
}

double MicrometeoriteImpact::GenerateMeteoriteDiameter() {
    std::mt19937 gen(static_cast<unsigned int>(m_RandomSeed++));
    std::exponential_distribution<double> dist(1.0);

    double diameter = dist(gen) * 1000.0;

    return std::min(diameter, 5000.0);
}

double MicrometeoriteImpact::CalculateCraterDiameter(double energyJoules) {
    if (energyJoules <= 0.0) {
        return 0.0;
    }

    double craterDiameter = CRATER_COEFFICIENT * std::pow(energyJoules, CRATER_EXPONENT);

    return craterDiameter;
}

int MicrometeoriteImpact::PoissonRandom(double lambda) {
    if (lambda <= 0.0) {
        return 0;
    }

    std::mt19937 gen(static_cast<unsigned int>(m_RandomSeed++));
    std::poisson_distribution<int> dist(lambda);

    return dist(gen);
}

}