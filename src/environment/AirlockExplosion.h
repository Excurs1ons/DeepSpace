#pragma once
#include "../engine/MockEngine.h"

namespace DeepSpace {
using Vec3d = Mock::Vec3d;

class AirlockExplosion {
public:
    AirlockExplosion();
    
    void TriggerExplosion(const Vec3d& center, double energy);
    
    double GetOverpressureAt(const Vec3d& position, double time) const;
    Vec3d GetTorqueFromAsymmetricDamage() const;
    
    double GetStructuralIntegrity() const { return m_StructuralIntegrity; }
    bool IsExploded() const { return m_Exploded; }
    
    void ApplyDamage(double damage);
    void Update(double dt);

private:
    Vec3d m_Center{0, 0, 0};
    double m_Energy = 0.0;
    double m_StructuralIntegrity = 1.0;
    double m_AsymmetricDamage = 0.0;
    bool m_Exploded = false;
    double m_TimeSinceExplosion = 0.0;
    
    static constexpr double OVERPRESSURE_DECAY_RATE = 0.5;
    static constexpr double DAMAGE_THRESHOLD = 0.3;
};

}
