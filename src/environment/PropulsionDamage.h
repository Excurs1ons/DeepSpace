#pragma once
#include "../engine/MockEngine.h"

namespace DeepSpace {

class PropulsionDamage {
public:
    PropulsionDamage();
    
    void ApplyDamage(double amount);
    void Update(double dt);
    
    double GetTotalDamage() const { return m_DamageLevel; }
    double GetThrustMultiplier() const { return 1.0 - m_DamageLevel * 0.8; }
    double GetFuelLeakRate() const { return m_DamageLevel * 0.5; }
    double GetVectoringError() const { return m_DamageLevel * 0.1; }
    
    bool IsCritical() const { return m_DamageLevel > 0.5; }

private:
    double m_DamageLevel = 0.0;
};

}