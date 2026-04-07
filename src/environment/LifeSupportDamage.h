#pragma once
#include "../engine/MockEngine.h"

namespace DeepSpace {

class LifeSupportDamage {
public:
    LifeSupportDamage();
    
    void SetCrewCount(int count) { m_CrewCount = count; }
    void ApplyDamage(double amount);
    void Update(double dt);
    
    double GetTotalDamage() const { return m_DamageLevel; }
    double GetCO2Level() const { return m_CO2Level; }
    double GetO2Level() const { return m_O2Level; }
    double GetCabinTemperature() const { return m_CabinTemperature; }
    double GetCabinPressure() const { return m_CabinPressure; }
    
    bool IsCritical() const;
    bool CheckCasualty() const;

private:
    double m_DamageLevel = 0.0;
    int m_CrewCount = 4;
    double m_CO2Level = 0.0004;
    double m_O2Level = 0.209;
    double m_CabinTemperature = 293.15;
    double m_CabinPressure = 101.325;
};

}
