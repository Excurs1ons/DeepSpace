#pragma once
#include "../engine/MockEngine.h"

namespace DeepSpace {

class TPSDamage {
public:
    TPSDamage();
    
    void ApplyImpact(double craterDiameter);
    void Update(double dt, double velocity, double density);
    
    double GetTotalDamage() const { return m_DamageLevel; }
    double GetHeatFluxMultiplier() const;
    double GetAblationRate() const { return m_AblationRate; }
    double GetSurfaceTemperature() const { return m_SurfaceTemperature; }
    double GetRemainingThickness() const { return m_RemainingThickness; }
    bool IsCritical() const { return m_DamageLevel > 0.5 || m_SurfaceTemperature > 1500.0; }
    
    static constexpr double NORMAL_THICKNESS = 0.025;
    static constexpr double CRITICAL_TEMP = 1800.0;

private:
    double m_DamageLevel = 0.0;
    double m_SurfaceTemperature = 293.15;
    double m_RemainingThickness = NORMAL_THICKNESS;
    double m_AblationRate = 0.0;
    
    static constexpr double CHAPMAN_C = 1.9e-4;
};

}