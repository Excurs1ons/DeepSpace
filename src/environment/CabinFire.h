#pragma once
#include <string>
#include <vector>
#include <map>

namespace DeepSpace {

enum class FireState {
    None,
    Smoldering,
    Active,
    Suppressed
};

class CabinFire {
public:
    CabinFire();
    
    void Ignite();
    void Update(double dt);
    void ActivateSuppression();
    
    FireState GetState() const { return m_State; }
    double GetTemperature() const { return m_Temperature; }
    double GetOxygenLevel() const { return m_OxygenLevel; }
    double GetSmokeDensity() const { return m_SmokeDensity; }
    bool IsSuppressionActive() const { return m_SuppressionActive; }
    
    void SetModuleVolume(double volume) { m_ModuleVolume = volume; }
    void SetCrewCount(int count) { m_CrewCount = count; }

private:
    FireState m_State = FireState::None;
    double m_Temperature = 293.15;
    double m_OxygenLevel = 0.21;
    double m_SmokeDensity = 0.0;
    bool m_SuppressionActive = false;
    double m_ModuleVolume = 100.0;
    int m_CrewCount = 4;
    
    static constexpr double O2_CONSUMPTION_PERSON = 0.000005;
    static constexpr double FIRE_TEMP_RISE_RATE = 5.0;
    static constexpr double SMOKE_RATE = 0.001;
    static constexpr double SUPPRESSION_COOLING_RATE = 2.0;
    static constexpr double SUPPRESSION_O2_RESTORE_RATE = 0.0001;
};
}