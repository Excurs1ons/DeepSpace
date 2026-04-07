#pragma once
#include <string>
#include <vector>
#include <map>

namespace DeepSpace {

enum class ModuleId { Bridge, Lab, Mess, Sleep, Cargo, Airlock };

class Depressurization {
public:
    Depressurization();
    
    void CreateLeak(double area);
    void SealBulkhead(ModuleId module);
    void OpenBulkhead(ModuleId module);
    
    double GetPressure(double time, double initialPressure = 101325.0) const;
    double GetTimeToUnconsciousness() const;
    double GetTimeToLethal() const;
    double GetCurrentPressure() const { return m_CurrentPressure; }
    
    void SetVolume(double volume) { m_Volume = volume; }
    void Update(double dt);
    
    bool IsBulkheadSealed(ModuleId module) const;
    double GetConductance() const { return m_Conductance; }

private:
    double m_Volume = 100.0;
    double m_LeakArea = 0.0;
    double m_Conductance = 0.0001;
    double m_CurrentPressure = 101325.0;
    std::map<ModuleId, bool> m_BulkheadSealed;
    
    static constexpr double ATMOSPHERIC_PRESSURE = 101325.0;
    static constexpr double CONSCIOUSNESS_THRESHOLD = 16000.0;
    static constexpr double LETHAL_THRESHOLD = 0.0;
};

}