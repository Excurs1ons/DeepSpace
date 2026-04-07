#include "Vessel.h"

namespace DeepSpace {

double Vessel::GetTotalDamage() const {
    return (m_TPSDamage + m_StructuralDamage + m_PropulsionDamage + m_LifeSupportDamage) / 4.0;
}

void Vessel::ApplyDamage(double amount, const Vec3d& location) {
    m_TPSDamage = std::min(1.0, m_TPSDamage + amount * 0.3);
    m_StructuralDamage = std::min(1.0, m_StructuralDamage + amount * 0.3);
    m_PropulsionDamage = std::min(1.0, m_PropulsionDamage + amount * 0.2);
    m_LifeSupportDamage = std::min(1.0, m_LifeSupportDamage + amount * 0.2);
}

void Vessel::UpdateWithDamage(double dt, double ambientPressure) {
    double thrustMultiplier = 1.0 - m_PropulsionDamage * 0.8;
    
    double massLossRate = m_StructuralDamage * 0.5;
    m_Body.SetMass(m_Body.GetMass() - massLossRate * dt);
    
    if (m_LifeSupportDamage > 0.0) {
        m_OxygenLevel -= m_LifeSupportDamage * 0.001 * dt;
        m_OxygenLevel = std::max(0.0, m_OxygenLevel);
        m_CabinTemperature += m_LifeSupportDamage * 0.5 * dt;
        m_CabinPressure -= m_LifeSupportDamage * 0.01 * dt;
        m_CabinPressure = std::max(0.0, m_CabinPressure);
    }
    
    (void)thrustMultiplier;
    (void)ambientPressure;
}

}
