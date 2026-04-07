#include "StructuralDamage.h"

namespace DeepSpace {

StructuralDamage::StructuralDamage()
    : m_DamageLevel(0.0)
    , m_AsymmetricVector(0, 0, 0)
{}

void StructuralDamage::ApplyDamage(double amount, const Vec3d& impactLocation) {
    m_DamageLevel = std::min(1.0, m_DamageLevel + amount);
    m_AsymmetricVector = m_AsymmetricVector + impactLocation.Normalized() * amount;
}

void StructuralDamage::Update(double dt) {
}

Vec3d StructuralDamage::GetAsymmetricTorque() const {
    return m_AsymmetricVector * m_DamageLevel * 1000.0;
}

}