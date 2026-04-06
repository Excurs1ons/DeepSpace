#pragma once
#include <vector>
#include <memory>
#include "../physics/PhysicsBody.h"
#include "Staging.h"
#include "RCS.h"

namespace DeepSpace {

    class Vessel {
    public:
        Vessel(const std::string& name) : m_Name(name), m_RCS(50000.0) {}

        void AddPart(std::shared_ptr<Part> part) {
            m_Parts.push_back(part);
            m_Staging.RebuildStages(m_Parts);
            RecalculateMass();
        }

        void ActivateNextStage() {
            m_Staging.ActivateNextStage();
            RecalculateMass();
        }

        void RecalculateMass() {
            double totalMass = 0.0;
            for (const auto& part : m_Parts) {
                totalMass += part->GetMass();
            }
            m_PhysicsBody.SetMass(totalMass);
        }

        void Update(double dt, double ambientPressure) {
            double totalThrustMagnitude = 0.0;

            // 1. Process engines
            for (auto& part : m_Parts) {
                part->Update(dt);
                
                if (auto engine = std::dynamic_pointer_cast<EnginePart>(part)) {
                    if (engine->IsActive() && engine->GetThrottle() > 0.0) {
                        double massFlow = engine->GetCurrentMassFlowRate() * dt;
                        
                        // Fuel consumption (only from active, non-decoupled tanks in the SAME or HIGHER stage)
                        // Mock: we just take fuel from any available non-decoupled tank for simplicity
                        bool hasFuel = false;
                        for (auto& p : m_Parts) {
                            if (auto tank = std::dynamic_pointer_cast<FuelTankPart>(p)) {
                                if (tank->GetStage() == engine->GetStage() && !tank->IsDecoupled() && tank->GetCurrentFuel() > 0) {
                                    // Simplified: try to consume all required fuel from this tank
                                    double available = tank->GetCurrentFuel();
                                    double toConsume = std::min(massFlow, available);
                                    tank->ConsumeFuel(toConsume);
                                    massFlow -= toConsume;
                                    if (massFlow <= 0.0001) {
                                        hasFuel = true;
                                        break;
                                    }
                                }
                            }
                        }

                        if (hasFuel || massFlow <= 0.0001) { // Still has some fuel to burn
                            totalThrustMagnitude += engine->GetThrust(ambientPressure);
                        } else {
                            engine->SetActive(false); 
                        }
                    }
                }
            }

            RecalculateMass();

            // 2. Apply thrust force using orientation
            Prisma::Vec3d thrustDirection = m_PhysicsBody.GetOrientation();
            Prisma::Vec3d thrustForce = thrustDirection * totalThrustMagnitude;
            
            m_PhysicsBody.AddForce(thrustForce);
        }

        PhysicsBody& GetPhysicsBody() { return m_PhysicsBody; }
        const PhysicsBody& GetPhysicsBody() const { return m_PhysicsBody; }
        
        const std::string& GetName() const { return m_Name; }
        const std::vector<std::shared_ptr<Part>>& GetParts() const { return m_Parts; }
        StagingSystem& GetStaging() { return m_Staging; }

    private:
        std::string m_Name;
        std::vector<std::shared_ptr<Part>> m_Parts;
        PhysicsBody m_PhysicsBody;
        StagingSystem m_Staging;
    };
}
      RCS m_RCS;
    };
}
