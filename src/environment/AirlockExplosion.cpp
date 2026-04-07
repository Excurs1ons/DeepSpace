#include "AirlockExplosion.h"
#include <cmath>

namespace DeepSpace {

AirlockExplosion::AirlockExplosion()
    : m_Center(0, 0, 0)
    , m_Energy(0.0)
    , m_StructuralIntegrity(1.0)
    , m_AsymmetricDamage(0.0)
    , m_Exploded(false)
    , m_TimeSinceExplosion(0.0)
{}

void AirlockExplosion::TriggerExplosion(const Vec3d& center, double energy) {
    m_Center = center;
    m_Energy = energy;
    m_Exploded = true;
    m_TimeSinceExplosion = 0.0;
}

double AirlockExplosion::GetOverpressureAt(const Vec3d& position, double time) const {
    if (!m_Exploded) return 0.0;
    
    double distance = (position - m_Center).Length();
    if (distance < 0.1) distance = 0.1;
    
    double pressure = m_Energy / (distance * distance);
    double decay = std::exp(-OVERPRESSURE_DECAY_RATE * time);
    return pressure * decay * 0.001;
}

Vec3d AirlockExplosion::GetTorqueFromAsymmetricDamage() const {
    if (m_AsymmetricDamage < 0.1) return {0, 0, 0};
    double torqueMagnitude = m_AsymmetricDamage * m_Energy * 0.001;
    return Vec3d{torqueMagnitude * 0.3, torqueMagnitude * 0.7, 0};
}

void AirlockExplosion::ApplyDamage(double damage) {
    m_StructuralIntegrity -= damage;
    m_StructuralIntegrity = std::max(0.0, m_StructuralIntegrity);
    
    m_AsymmetricDamage += damage * 0.5;
    m_AsymmetricDamage = std::min(1.0, m_AsymmetricDamage);
}

void AirlockExplosion::Update(double dt) {
    if (!m_Exploded) return;
    m_TimeSinceExplosion += dt;
}

}
