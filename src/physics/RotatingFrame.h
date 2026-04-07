#pragma once
#include "../core/Quaternion.h"

namespace DeepSpace {

class RotatingFrame {
public:
    RotatingFrame();
    RotatingFrame(const Vec3d& origin, const Vec3d& angularVelocity);
    
    void SetOrigin(const Vec3d& origin);
    void SetAngularVelocity(const Vec3d& omega);
    void SetAngularVelocityRpm(double rpm);
    
    const Vec3d& GetOrigin() const { return m_Origin; }
    const Vec3d& GetAngularVelocity() const { return m_AngularVelocity; }
    double GetAngularVelocityRpm() const;
    
    Vec3d ToRotating(const Vec3d& inertialPos, double time) const;
    Vec3d VelocityToRotating(const Vec3d& inertialVel, const Vec3d& inertialPos, double time) const;
    
    Vec3d ToInertial(const Vec3d& rotatingPos, double time) const;
    Vec3d VelocityToInertial(const Vec3d& rotatingVel, const Vec3d& rotatingPos, double time) const;
    
    Vec3d GetArtificialGravity(const Vec3d& localPos) const;
    
    static constexpr double NORMAL_RPM = 5.6;
    static constexpr double EMERGENCY_RPM = 68.0;
    static constexpr double NORMAL_OMEGA = 0.586;
    static constexpr double EMERGENCY_OMEGA = 7.11;

private:
    Vec3d m_Origin;
    Vec3d m_AngularVelocity;
    Quaternion m_Orientation;
    double m_InitialAngle;
};

}