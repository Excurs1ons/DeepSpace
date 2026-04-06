#pragma once
#include <PrismaEngine.h>
#include "../physics/PhysicsBody.h"

namespace DeepSpace {

    class RCS {
    public:
        RCS(double power) : m_Power(power), m_Enabled(false) {}

        void SetEnabled(bool enabled) { m_Enabled = enabled; }
        bool IsEnabled() const { return m_Enabled; }

        // Apply rotation torque based on input (-1, 0, 1)
        void ApplyRotation(PhysicsBody& body, double input, double dt) {
            if (!m_Enabled || std::abs(input) < 0.01) return;
            
            // Torque = Force * lever_arm (Simplified: we just use power as torque)
            body.AddTorque(input * m_Power);
        }

        // Apply translation (for docking/precision)
        void ApplyTranslation(PhysicsBody& body, const Prisma::Vec3d& localDir, double dt) {
            if (!m_Enabled || localDir.Length() < 0.01) return;

            // Convert local direction to world direction based on orientation
            Prisma::Vec3d orientation = body.GetOrientation();
            
            // Simple 2D rotation for translation
            // In 2D: local forward is orientation, local right is perp to orientation
            Prisma::Vec3d worldForce;
            if (localDir.y > 0) worldForce += orientation * m_Power; // Forward
            if (localDir.y < 0) worldForce -= orientation * m_Power; // Backward
            
            Prisma::Vec3d right(-orientation.y, orientation.x, 0.0);
            if (localDir.x > 0) worldForce += right * m_Power; // Right
            if (localDir.x < 0) worldForce -= right * m_Power; // Left

            body.AddForce(worldForce);
        }

    private:
        double m_Power;
        bool m_Enabled;
    };
}
