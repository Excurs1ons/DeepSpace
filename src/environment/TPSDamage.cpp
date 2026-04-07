#include "TPSDamage.h"
#include <cmath>

namespace DeepSpace {

TPSDamage::TPSDamage()
    : m_DamageLevel(0.0)
    , m_SurfaceTemperature(293.15)
    , m_RemainingThickness(NORMAL_THICKNESS)
    , m_AblationRate(0.0)
{}

void TPSDamage::ApplyImpact(double craterDiameter) {
    double areaRatio = craterDiameter / (1.0);
    m_DamageLevel = std::min(1.0, m_DamageLevel + areaRatio * 0.5);
}

void TPSDamage::Update(double dt, double velocity, double density) {
    if (velocity < 100.0 || density < 1e-10) return;
    
    double v3 = velocity * velocity * velocity;
    double q_conv = CHAPMAN_C * std::sqrt(density) * v3;
    
    double heatMult = GetHeatFluxMultiplier();
    double q_effective = q_conv * heatMult;
    
    m_SurfaceTemperature += q_effective * dt / 10000.0;
    m_SurfaceTemperature = std::min(m_SurfaceTemperature, 3000.0);
    
    if (m_SurfaceTemperature > 500.0) {
        double ablationEnthalpy = 50e6;
        m_AblationRate = q_effective / ablationEnthalpy;
        m_RemainingThickness -= m_AblationRate * dt / 256.0;
        m_RemainingThickness = std::max(0.0, m_RemainingThickness);
    }
    
    if (m_RemainingThickness < NORMAL_THICKNESS * 0.2) {
        m_DamageLevel = 1.0;
    }
}

double TPSDamage::GetHeatFluxMultiplier() const {
    return 1.0 + m_DamageLevel * 3.0;
}

}