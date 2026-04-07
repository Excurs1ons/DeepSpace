#pragma once
#include "../engine/MockEngine.h"

namespace DeepSpace {
using Vec3d = Mock::Vec3d;

class Coriolis {
public:
    static Vec3d CalculateForce(const Vec3d& angularVelocity, const Vec3d& velocityInRotatingFrame, double mass = 1.0) {
        Vec3d omegaCrossV = Vec3d::Cross(angularVelocity, velocityInRotatingFrame);
        return omegaCrossV * (-2.0 * mass);
    }

    static Vec3d CalculateAcceleration(const Vec3d& angularVelocity, const Vec3d& velocityInRotatingFrame) {
        Vec3d omegaCrossV = Vec3d::Cross(angularVelocity, velocityInRotatingFrame);
        return omegaCrossV * (-2.0);
    }

    static Vec3d CalculateDeflection(const Vec3d& angularVelocity, const Vec3d& velocityInRotatingFrame, double dt) {
        Vec3d acceleration = CalculateAcceleration(angularVelocity, velocityInRotatingFrame);
        return acceleration * dt * dt * 0.5;
    }
};

}