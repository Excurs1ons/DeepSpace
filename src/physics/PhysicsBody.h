#pragma once
#include <cmath>
#include "../engine/MockEngine.h"
#include "../core/Quaternion.h"
#include "../core/Mat3x3.h"

namespace DeepSpace {
using Vec3d = Mock::Vec3d;

    class PhysicsBody {
    public:
        PhysicsBody()
            : m_Position(0.0, 0.0, 0.0),
              m_Velocity(0.0, 0.0, 0.0),
              m_AccumulatedForce(0.0, 0.0, 0.0),
              m_Mass(1000.0),
              m_Orientation{1.0, 0.0, 0.0, 0.0},
              m_AngularVelocity(0.0, 0.0, 0.0),
              m_AccumulatedTorque(0.0, 0.0, 0.0),
              m_Inertia(1000.0) {}

        void AddForce(const Vec3d& force) {
            m_AccumulatedForce += force;
        }
        
        Vec3d GetAccumulatedForce() const { return m_AccumulatedForce; }

        void AddTorque(double torque) {
            m_AccumulatedTorque.z += torque;
        }

        void AddTorque3D(const Vec3d& torque) {
            m_AccumulatedTorque += torque;
        }

        void Update(double dt) {
            if (m_Mass <= 0.0 || dt <= 0.0) {
                return;
            }

            const Vec3d acceleration = m_AccumulatedForce / m_Mass;
            m_Velocity += acceleration * dt;
            m_Position += m_Velocity * dt;

            const double angularAcceleration = (m_Inertia > 0.0) ? (m_AccumulatedTorque.Length() / m_Inertia) : 0.0;
            m_AngularVelocity += m_AccumulatedTorque * (1.0 / m_Inertia) * dt;

            const double angle = m_AngularVelocity.Length() * dt;
            if (angle > 1e-10) {
                Vec3d axis = m_AngularVelocity.Normalized();
                Quaternion dq = Quaternion::FromAxisAngle(axis, angle);
                m_Orientation = (m_Orientation * dq).Normalized();
            }

            m_AccumulatedForce = {0.0, 0.0, 0.0};
            m_AccumulatedTorque = {0.0, 0.0, 0.0};
        }

        void SetPosition(const Vec3d& pos) { m_Position = pos; }
        const Vec3d& GetPosition() const { return m_Position; }

        void SetVelocity(const Vec3d& vel) { m_Velocity = vel; }
        const Vec3d& GetVelocity() const { return m_Velocity; }

        void SetOrientation(const Vec3d& dir) {
            double angle = -std::atan2(dir.x, dir.y);
            m_Orientation = Quaternion::FromAxisAngle({0, 0, 1}, angle);
        }
        const Quaternion& GetOrientation() const { return m_Orientation; }

        Vec3d GetOrientationVec3() const {
            return m_Orientation * Vec3d(0, 1, 0);
        }

        void SetAngularVelocity(double w) { m_AngularVelocity = {0, 0, w}; }
        double GetAngularVelocity() const { return m_AngularVelocity.z; }

        Vec3d GetAngularVelocity3D() const { return m_AngularVelocity; }

        void SetMass(double mass) { m_Mass = mass; }
        double GetMass() const { return m_Mass; }

        void SetInertia(double inertia) { m_Inertia = inertia; }

    private:
        Vec3d m_Position;
        Vec3d m_Velocity;
        Vec3d m_AccumulatedForce;
        double m_Mass;

        Quaternion m_Orientation;
        Vec3d m_AngularVelocity;
        Vec3d m_AccumulatedTorque;
        double m_Inertia;
    };
}
