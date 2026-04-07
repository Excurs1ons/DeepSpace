#pragma once
#include "../engine/MockEngine.h"

namespace DeepSpace {
using Vec3d = Mock::Vec3d;

class StructuralDamage {
public:
    StructuralDamage();
    
    void ApplyDamage(double amount, const Vec3d& impactLocation);
    void Update(double dt);
    
    double GetTotalDamage() const { return m_DamageLevel; }
    double GetDragMultiplier() const { return 1.0 + m_DamageLevel * 0.5; }
    double GetInertiaMultiplier() const { return 1.0 + m_DamageLevel * 0.3; }
    Vec3d GetAsymmetricTorque() const;
    
    bool IsCritical() const { return m_DamageLevel > 0.8; }

private:
    double m_DamageLevel = 0.0;
    Vec3d m_AsymmetricVector{0, 0, 0};
};

}