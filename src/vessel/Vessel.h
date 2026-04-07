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
            for (auto& part : m_Parts) {
                part->Update(dt);
                if (auto engine = std::dynamic_pointer_cast<EnginePart>(part)) {
                    if (engine->IsActive() && engine->GetThrottle() > 0.0) {
                        double massFlow = engine->GetCurrentMassFlowRate() * dt;
                        bool hasFuel = false;
                        for (auto& p : m_Parts) {
                            if (auto tank = std::dynamic_pointer_cast<FuelTankPart>(p)) {
                                if (tank->GetStage() == engine->GetStage() && !tank->IsDecoupled() && tank->GetCurrentFuel() > 0) {
                                    double toConsume = std::min(massFlow, tank->GetCurrentFuel());
                                    tank->ConsumeFuel(toConsume);
                                    massFlow -= toConsume;
                                    if (massFlow <= 0.0001) { hasFuel = true; break; }
                                }
                            }
                        }
                        if (hasFuel || massFlow <= 0.0001) totalThrustMagnitude += engine->GetThrust(ambientPressure);
                        else engine->SetActive(false);
                    }
                }
            }
            RecalculateMass();
            m_PhysicsBody.AddForce(m_PhysicsBody.GetOrientation() * totalThrustMagnitude);
        }

        PhysicsBody& GetPhysicsBody() { return m_PhysicsBody; }
        const std::string& GetName() const { return m_Name; }
        const std::vector<std::shared_ptr<Part>>& GetParts() const { return m_Parts; }
        StagingSystem& GetStaging() { return m_Staging; }
        RCS& GetRCS() { return m_RCS; }

    private:
        std::string m_Name;
        std::vector<std::shared_ptr<Part>> m_Parts;
        PhysicsBody m_PhysicsBody;
        StagingSystem m_Staging;
        RCS m_RCS;
    };
}
