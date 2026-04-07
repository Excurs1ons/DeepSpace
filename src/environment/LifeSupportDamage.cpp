#include "LifeSupportDamage.h"

namespace DeepSpace {

LifeSupportDamage::LifeSupportDamage()
    : m_DamageLevel(0.0)
    , m_CrewCount(4)
    , m_CO2Level(0.0004)
    , m_O2Level(0.209)
    , m_CabinTemperature(293.15)
    , m_CabinPressure(101.325)
{}

void LifeSupportDamage::ApplyDamage(double amount) {
    m_DamageLevel = std::min(1.0, m_DamageLevel + amount);
}

void LifeSupportDamage::Update(double dt) {
    double scrubberEfficiency = 1.0 - m_DamageLevel;
    double co2Generation = 0.008 / 3600.0 * m_CrewCount;
    m_CO2Level += co2Generation * dt * (1.0 / scrubberEfficiency);
    m_CO2Level = std::min(1.0, m_CO2Level);
    
    double o2Gen = 5.5 / 60000.0 * m_CrewCount;
    m_O2Level += o2Gen * scrubberEfficiency * dt;
    m_O2Level = std::min(0.5, m_O2Level);
    
    m_CabinTemperature += m_DamageLevel * 0.1 * dt;
    m_CabinTemperature = std::min(350.0, m_CabinTemperature);
    
    m_CabinPressure -= m_DamageLevel * 0.05 * dt;
    m_CabinPressure = std::max(0.0, m_CabinPressure);
}

bool LifeSupportDamage::IsCritical() const {
    return m_CO2Level > 0.04 || m_O2Level < 0.16 || 
           m_CabinTemperature > 310.0 || m_CabinPressure < 70.0;
}

bool LifeSupportDamage::CheckCasualty() const {
    return m_CO2Level > 0.1 || m_O2Level < 0.10 || 
           m_CabinTemperature > 330.0 || m_CabinPressure < 50.0;
}

}
