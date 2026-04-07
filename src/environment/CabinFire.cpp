#include "CabinFire.h"
#include <algorithm>

namespace DeepSpace {

CabinFire::CabinFire()
    : m_State(FireState::None)
    , m_Temperature(293.15)
    , m_OxygenLevel(0.21)
    , m_SmokeDensity(0.0)
    , m_SuppressionActive(false)
    , m_ModuleVolume(100.0)
    , m_CrewCount(4)
{}

void CabinFire::Ignite() {
    if (m_State == FireState::None) {
        m_State = FireState::Smoldering;
    }
}

void CabinFire::Update(double dt) {
    if (m_State == FireState::None || m_State == FireState::Suppressed) return;
    
    if (m_State == FireState::Smoldering) {
        m_Temperature += 1.0 * dt;
        if (m_Temperature > 400.0) {
            m_State = FireState::Active;
        }
    }
    
    if (m_State == FireState::Active) {
        m_Temperature += FIRE_TEMP_RISE_RATE * dt;
        m_Temperature = std::min(m_Temperature, 1200.0);
        
        double oxygenConsumption = (m_CrewCount * O2_CONSUMPTION_PERSON) + 0.00001;
        m_OxygenLevel -= oxygenConsumption * dt / m_ModuleVolume;
        m_OxygenLevel = std::max(m_OxygenLevel, 0.0);
        
        if (m_OxygenLevel < 0.05) {
            m_State = FireState::Smoldering;
        }
        
        m_SmokeDensity += SMOKE_RATE * dt;
        m_SmokeDensity = std::min(m_SmokeDensity, 1.0);
    }
    
    if (m_SuppressionActive) {
        m_Temperature -= SUPPRESSION_COOLING_RATE * dt;
        m_Temperature = std::max(m_Temperature, 293.15);
        
        m_OxygenLevel += SUPPRESSION_O2_RESTORE_RATE * dt;
        m_OxygenLevel = std::min(m_OxygenLevel, 0.21);
        
        if (m_Temperature <= 320.0 && m_SmokeDensity < 0.3) {
            m_State = FireState::Suppressed;
            m_SuppressionActive = false;
        }
    }
}

void CabinFire::ActivateSuppression() {
    m_SuppressionActive = true;
    if (m_State == FireState::Smoldering || m_State == FireState::Active) {
        m_State = FireState::Active;
    }
}

}