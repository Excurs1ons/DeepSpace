#pragma once
#include "../engine/MockEngine.h"

namespace DeepSpace {

class ThermalSimulation {
public:
    ThermalSimulation();
    
    void Update(double dt, double velocity, double density, double tpsIntegrity);
    
    double GetSurfaceTemperature() const { return m_SurfaceTemperature; }
    double GetAblationRate() const { return m_AblationRate; }
    double GetHeatShieldIntegrity() const;
    bool IsSurvivable() const;
    double GetCrewSurvivalProbability() const;
    void SimulateReentry(double peakDeceleration, double totalHeatLoad);

private:
    double m_SurfaceTemperature = 293.15;
    double m_AblationRate = 0.0;
    double m_TotalHeatLoad = 0.0;
    double m_AblatedMass = 0.0;
    double m_CharLayerThickness = 0.0;
    double m_TPSThickness = 0.025;
    double m_ReentrySeverity = 0.0;
    double m_PeakDeceleration = 0.0;
    
    static constexpr double STAGNATION_POINT_FACTOR = 1.3;
    static constexpr double ABLATION_COEFFICIENT = 3.0e-4;
    static constexpr double CHARR_LAYER_DENSITY = 300.0;
    static constexpr double PRANDTL_NUMBER = 0.71;
    static constexpr double SPECIFIC_HEAT = 1005.0;
    static constexpr double SUBLIMATION_ENTHALPY = 50e6;
    static constexpr double LATENT_HEAT = 2.5e6;
};

}