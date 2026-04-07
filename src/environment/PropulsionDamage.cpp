#include "PropulsionDamage.h"

namespace DeepSpace {

PropulsionDamage::PropulsionDamage()
    : m_DamageLevel(0.0)
{}

void PropulsionDamage::ApplyDamage(double amount) {
    m_DamageLevel = std::min(1.0, m_DamageLevel + amount);
}

void PropulsionDamage::Update(double dt) {
}

}