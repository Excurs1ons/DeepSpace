#include "Depressurization.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace DeepSpace {

Depressurization::Depressurization()
    : m_Volume(100.0)
    , m_LeakArea(0.0)
    , m_Conductance(0.0001)
    , m_CurrentPressure(ATMOSPHERIC_PRESSURE)
{
    for (int i = static_cast<int>(ModuleId::Bridge); i <= static_cast<int>(ModuleId::Airlock); ++i) {
        m_BulkheadSealed[static_cast<ModuleId>(i)] = false;
    }
}

void Depressurization::CreateLeak(double area) {
    m_LeakArea = area;
    m_Conductance = 0.0001 * (area / 0.001);
}

void Depressurization::SealBulkhead(ModuleId module) {
    m_BulkheadSealed[module] = true;
}

void Depressurization::OpenBulkhead(ModuleId module) {
    m_BulkheadSealed[module] = false;
}

double Depressurization::GetPressure(double time, double initialPressure) const {
    if (m_LeakArea <= 0.0) return initialPressure;
    double k = m_Conductance * m_LeakArea / m_Volume;
    return initialPressure * std::exp(-k * time);
}

double Depressurization::GetTimeToUnconsciousness() const {
    if (m_LeakArea <= 0.0) return std::numeric_limits<double>::infinity();
    double k = m_Conductance * m_LeakArea / m_Volume;
    return std::log(CONSCIOUSNESS_THRESHOLD / ATMOSPHERIC_PRESSURE) / (-k);
}

double Depressurization::GetTimeToLethal() const {
    if (m_LeakArea <= 0.0) return std::numeric_limits<double>::infinity();
    double k = m_Conductance * m_LeakArea / m_Volume;
    return std::log(LETHAL_THRESHOLD / ATMOSPHERIC_PRESSURE + 1.0) / (-k);
}

void Depressurization::Update(double dt) {
    if (m_LeakArea > 0.0) {
        double k = m_Conductance * m_LeakArea / m_Volume;
        m_CurrentPressure *= std::exp(-k * dt);
        m_CurrentPressure = std::max(m_CurrentPressure, 0.0);
    }
}

bool Depressurization::IsBulkheadSealed(ModuleId module) const {
    auto it = m_BulkheadSealed.find(module);
    return it != m_BulkheadSealed.end() && it->second;
}

}