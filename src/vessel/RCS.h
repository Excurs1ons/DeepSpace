#pragma once
#include <PrismaEngine.h>
#include "../physics/PhysicsBody.h"
#include <cmath>

namespace DeepSpace {

    class RCS {
    public:
        explicit RCS(double power) : m_Power(power), m_Enabled(false) {}

        void SetEnabled(bool enabled) { m_Enabled = enabled; }
        bool IsEnabled() const { return m_Enabled; }

        void ApplyRotation(PhysicsBody& body, double input, double dt) {
            if (!m_Enabled || std::abs(input) < 0.01 || dt <= 0.0) return;
            body.AddTorque(input * m_Power);
        }

        void ApplyTranslation(PhysicsBody& body, const Prisma::Vec3d& localDir, double dt) {
            if (!m_Enabled || localDir.Length() < 0.01 || dt <= 0.0) return;

            const Prisma::Vec3d orientation = body.GetOrientation();
            Prisma::Vec3d worldForce;
            if (localDir.y > 0) worldForce += orientation * m_Power;
            if (localDir.y < 0) worldForce -= orientation * m_Power;

            const Prisma::Vec3d right(-orientation.y, orientation.x, 0.0);
            if (localDir.x > 0) worldForce += right * m_Power;
            if (localDir.x < 0) worldForce -= right * m_Power;

            body.AddForce(worldForce);
        }

        void Stabilize(PhysicsBody& body, double dt) {
            if (!m_Enabled || dt <= 0.0) return;
            const double damping = 0.98;
            body.SetAngularVelocity(body.GetAngularVelocity() * damping);
        }

    private:
        double m_Power;
        bool m_Enabled;
    };
}
