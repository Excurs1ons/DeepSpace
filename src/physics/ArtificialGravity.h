#pragma once
#include "../engine/MockEngine.h"
#include <cmath>

namespace DeepSpace {

class ArtificialGravity {
public:
    static constexpr double NORMAL_RPM = 5.6;
    static constexpr double EMERGENCY_RPM = 68.0;
    static constexpr double NORMAL_OMEGA = 0.586;
    static constexpr double EMERGENCY_OMEGA = 7.11;

    static double RpmToRadS(double rpm) {
        return rpm * 2.0 * M_PI / 60.0;
    }

    static double RadSToRpm(double radS) {
        return radS * 60.0 / (2.0 * M_PI);
    }

    static double CalculateGravityAtRadius(double radius, double omega) {
        return omega * omega * radius;
    }

    static double CalculateGravityAtRadiusRpm(double radius, double rpm) {
        double omega = RpmToRadS(rpm);
        return omega * omega * radius;
    }

    static Mock::Vec3d CalculateCentripetalAcceleration(
        const Mock::Vec3d& positionRelativeToAxis,
        const Mock::Vec3d& angularVelocity) {
        Mock::Vec3d omegaCrossR = Mock::Vec3d::Cross(angularVelocity, positionRelativeToAxis);
        return Mock::Vec3d::Cross(angularVelocity, omegaCrossR);
    }

    static double GetRadiusFromGravity(double targetGravity, double omega) {
        if (omega <= 0.0) return 0.0;
        return targetGravity / (omega * omega);
    }

    static double GetRpmFromGravityAndRadius(double targetGravity, double radius) {
        if (radius <= 0.0) return 0.0;
        return RadSToRpm(std::sqrt(targetGravity / radius));
    }
};

}