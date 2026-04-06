#pragma once
#include <PrismaEngine.h>
#include <vector>

namespace DeepSpace {

    class PhysicsBody {
    public:
        PhysicsBody() : m_Position(0,0,0), m_Velocity(0,0,0), m_Mass(1000.0), 
                        m_Orientation(0,1,0), m_AngularVelocity(0), m_Inertia(1000.0) {}

        void AddForce(const Prisma::Vec3d& force) {
            m_AccumulatedForce += force;
        }

        void AddTorque(double torque) {
            m_AccumulatedTorque += torque;
        }

        void Update(double dt) {
            if (m_Mass <= 0.0) return;

            // Linear Integration
            Prisma::Vec3d acceleration = m_AccumulatedForce / m_Mass;
            m_Velocity += acceleration * dt;
            m_Position += m_Velocity * dt;

            // Rotational Integration (Simplified 2D-in-3D: rotating around Z axis)
            double angularAcceleration = m_AccumulatedTorque / m_Inertia;
            m_AngularVelocity += angularAcceleration * dt;
            
            // Rotate orientation vector (Simplified 2D rotation)
            double angle = m_AngularVelocity * dt;
            double cosA = std::cos(angle);
            double sinA = std::sin(angle);
            double nx = m_Orientation.x * cosA - m_Orientation.y * sinA;
            double ny = m_Orientation.x * sinA + m_Orientation.y * cosA;
            m_Orientation = {nx, ny, 0.0};
            m_Orientation = m_Orientation.Normalized();

            // Clear accumulators
            m_AccumulatedForce = {0, 0, 0};
            m_AccumulatedTorque = 0.0;
        }

        // Getters and Setters
        void SetPosition(const Prisma::Vec3d& pos) { m_Position = pos; }
        const Prisma::Vec3d& GetPosition() const { return m_Position; }
        
        void SetVelocity(const Prisma::Vec3d& vel) { m_Velocity = vel; }
        const Prisma::Vec3d& GetVelocity() const { return m_Velocity; }

        void SetOrientation(const Prisma::Vec3d& dir) { m_Orientation = dir.Normalized(); }
        const Prisma::Vec3d& GetOrientation() const { return m_Orientation; }

        void SetAngularVelocity(double w) { m_AngularVelocity = w; }
        double GetAngularVelocity() const { return m_AngularVelocity; }

        void SetMass(double mass) { m_Mass = mass; }
        double GetMass() const { return m_Mass; }
        
        void SetInertia(double inertia) { m_Inertia = inertia; }

    private:
        Prisma::Vec3d m_Position;
        Prisma::Vec3d m_Velocity;
        Prisma::Vec3d m_AccumulatedForce;
        double m_Mass;

        // Rotation
        Prisma::Vec3d m_Orientation; // Looking at (x,y,z)
        double m_AngularVelocity;
        double m_AccumulatedTorque;
        double m_Inertia;
    };
}
