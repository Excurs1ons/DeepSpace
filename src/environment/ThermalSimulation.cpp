#include "ThermalSimulation.h"
#include <cmath>

namespace DeepSpace {

ThermalSimulation::ThermalSimulation()
    : m_SurfaceTemperature(293.15)
    , m_AblationRate(0.0)
    , m_TotalHeatLoad(0.0)
    , m_AblatedMass(0.0)
    , m_CharLayerThickness(0.0)
    , m_TPSThickness(0.025)
    , m_ReentrySeverity(0.0)
    , m_PeakDeceleration(0.0)
{
}

void ThermalSimulation::Update(double dt, double velocity, double density, double tpsIntegrity) {
    if (velocity < 1.0 || density < 1e-15 || dt <= 0.0) {
        return;
    }
    
    double v3 = velocity * velocity * velocity;
    
    double q_stag = 0.5 * density * v3 * SPECIFIC_HEAT / std::pow(PRANDTL_NUMBER, 0.6);
    q_stag *= STAGNATION_POINT_FACTOR;
    
    m_TotalHeatLoad += q_stag * dt;
    
    if (m_SurfaceTemperature > 500.0) {
        double totalEnthalpy = SUBLIMATION_ENTHALPY + LATENT_HEAT;
        double heatFluxEffective = q_stag * (1.0 + (1.0 - tpsIntegrity) * ABLATION_COEFFICIENT);
        
        m_AblationRate = heatFluxEffective / totalEnthalpy;
        m_AblatedMass += m_AblationRate * dt * CHARR_LAYER_DENSITY;
        
        m_TPSThickness -= m_AblationRate * dt;
        m_TPSThickness = std::max(0.0, m_TPSThickness);
        
        double charDensityRatio = m_AblatedMass / (m_TPSThickness + 1e-10);
        m_CharLayerThickness = std::min(m_CharLayerThickness + m_AblationRate * dt * 0.5, 0.02);
    }
    
    double tempRise = q_stag * dt / 8000.0;
    m_SurfaceTemperature += tempRise;
    m_SurfaceTemperature = std::min(m_SurfaceTemperature, 3000.0);
    
    m_ReentrySeverity = m_TotalHeatLoad / 10e6;
    m_PeakDeceleration = std::max(m_PeakDeceleration, velocity / 100.0);
}

double ThermalSimulation::GetHeatShieldIntegrity() const {
    double initialThickness = 0.025;
    double integrity = m_TPSThickness / initialThickness;
    return std::max(0.0, std::min(1.0, integrity));
}

bool ThermalSimulation::IsSurvivable() const {
    return GetHeatShieldIntegrity() > 0.15 && m_SurfaceTemperature < 2000.0;
}

double ThermalSimulation::GetCrewSurvivalProbability() const {
    double integrity = GetHeatShieldIntegrity();
    double baseSurvival = 0.0;
    
    if (integrity > 0.8) {
        baseSurvival = 1.0;
    } else if (integrity > 0.5) {
        baseSurvival = 1.0 - (0.8 - integrity) / 0.3 * 0.5;
    } else if (integrity > 0.2) {
        baseSurvival = 0.5 - (0.5 - integrity) / 0.3 * 0.5;
    } else {
        baseSurvival = 0.0;
    }
    
    double gForceRisk = 0.0;
    if (m_PeakDeceleration > 8.0) {
        gForceRisk = (m_PeakDeceleration - 8.0) * 0.01;
        gForceRisk = std::min(gForceRisk, 0.1);
    }
    
    double finalSurvival = baseSurvival - gForceRisk;
    return std::max(0.0, std::min(1.0, finalSurvival));
}

void ThermalSimulation::SimulateReentry(double peakDeceleration, double totalHeatLoad) {
    m_PeakDeceleration = peakDeceleration;
    m_TotalHeatLoad = totalHeatLoad;
    
    m_ReentrySeverity = totalHeatLoad / 10e6;
    
    double baseIntegrity = 1.0 - (m_ReentrySeverity * 0.8);
    baseIntegrity = std::max(0.0, std::min(1.0, baseIntegrity));
    
    double gForceDamage = 0.0;
    if (peakDeceleration > 8.0) {
        gForceDamage = (peakDeceleration - 8.0) * 0.05;
        gForceDamage = std::min(gForceDamage, 0.3);
    }
    
    m_TPSThickness = 0.025 * (baseIntegrity - gForceDamage);
    m_TPSThickness = std::max(0.0, m_TPSThickness);
    
    if (totalHeatLoad > 5e6) {
        double ablationFraction = (totalHeatLoad - 5e6) / 15e6;
        double ablationAmount = ablationFraction * 0.015;
        m_TPSThickness -= ablationAmount;
        m_TPSThickness = std::max(0.0, m_TPSThickness);
        
        m_CharLayerThickness = std::min(ablationAmount * 2.0, 0.02);
        m_AblatedMass = (0.015 - m_TPSThickness) * CHARR_LAYER_DENSITY;
    }
    
    m_SurfaceTemperature = 293.15 + (totalHeatLoad / 10e6) * 1000.0;
    m_SurfaceTemperature = std::min(m_SurfaceTemperature, 3000.0);
}

}