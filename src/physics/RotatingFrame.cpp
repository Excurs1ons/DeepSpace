#include "RotatingFrame.h"
#include <cmath>

namespace DeepSpace {

RotatingFrame::RotatingFrame()
    : m_Origin(0, 0, 0)
    , m_AngularVelocity(0, 0, 0)
    , m_Orientation()
    , m_InitialAngle(0.0)
{}

RotatingFrame::RotatingFrame(const Vec3d& origin, const Vec3d& angularVelocity)
    : m_Origin(origin)
    , m_AngularVelocity(angularVelocity)
    , m_Orientation()
    , m_InitialAngle(0.0)
{}

void RotatingFrame::SetOrigin(const Vec3d& origin) {
    m_Origin = origin;
}

void RotatingFrame::SetAngularVelocity(const Vec3d& omega) {
    m_AngularVelocity = omega;
}

void RotatingFrame::SetAngularVelocityRpm(double rpm) {
    const double omega = rpm * 2.0 * M_PI / 60.0;
    m_AngularVelocity = {0, 0, omega};
}

double RotatingFrame::GetAngularVelocityRpm() const {
    return m_AngularVelocity.z * 60.0 / (2.0 * M_PI);
}

Vec3d RotatingFrame::ToRotating(const Vec3d& inertialPos, double time) const {
    Vec3d rel = inertialPos - m_Origin;
    double angle = m_AngularVelocity.Length() * time + m_InitialAngle;
    Vec3d axis = m_AngularVelocity.Normalized();
    Quaternion rot = Quaternion::FromAxisAngle(axis, -angle);
    return rot * rel;
}

Vec3d RotatingFrame::VelocityToRotating(const Vec3d& inertialVel, const Vec3d& inertialPos, double time) const {
    Vec3d rel = inertialPos - m_Origin;
    double angle = m_AngularVelocity.Length() * time + m_InitialAngle;
    Vec3d axis = m_AngularVelocity.Normalized();
    Quaternion rot = Quaternion::FromAxisAngle(axis, -angle);
    return rot * inertialVel - Vec3d::Cross(m_AngularVelocity, rot * rel);
}

Vec3d RotatingFrame::ToInertial(const Vec3d& rotatingPos, double time) const {
    double angle = m_AngularVelocity.Length() * time + m_InitialAngle;
    Vec3d axis = m_AngularVelocity.Normalized();
    Quaternion rot = Quaternion::FromAxisAngle(axis, angle);
    return m_Origin + (rot * rotatingPos);
}

Vec3d RotatingFrame::VelocityToInertial(const Vec3d& rotatingVel, const Vec3d& rotatingPos, double time) const {
    double angle = m_AngularVelocity.Length() * time + m_InitialAngle;
    Vec3d axis = m_AngularVelocity.Normalized();
    Quaternion rot = Quaternion::FromAxisAngle(axis, angle);
    return rot * rotatingVel + Vec3d::Cross(m_AngularVelocity, rot * rotatingPos);
}

Vec3d RotatingFrame::GetArtificialGravity(const Vec3d& localPos) const {
    Vec3d r = localPos;
    Vec3d omegaCrossR = Vec3d::Cross(m_AngularVelocity, r);
    Vec3d omegaCrossOmegaCrossR = Vec3d::Cross(m_AngularVelocity, omegaCrossR);
    return omegaCrossOmegaCrossR;
}

}