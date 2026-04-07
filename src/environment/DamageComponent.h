#pragma once
#include "../physics/PhysicsBody.h"

namespace DeepSpace {

class DamageComponent {
public:
    virtual ~DamageComponent() = default;
    
    virtual double GetTotalDamage() const = 0;
    virtual void ApplyDamage(double amount, const Vec3d& location = {0, 0, 0}) = 0;
    virtual void Update(double dt, PhysicsBody& body) = 0;
    
    bool IsCritical() const { return GetTotalDamage() > 0.8; }
    bool IsDestroyed() const { return GetTotalDamage() >= 1.0; }

protected:
    double m_DamageLevel = 0.0;
};

}
