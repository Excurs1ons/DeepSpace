#pragma once
#include <algorithm>
#include <memory>
#include <vector>
#include "../physics/PhysicsBody.h"
#include "RCS.h"
#include "Staging.h"

namespace DeepSpace {

    struct EngineStatus {
        int activeEngines = 0;
        double totalThrust = 0.0;
        double totalMassFlow = 0.0;
        double totalFuelFlow = 0.0;
        double totalOxidizerFlow = 0.0;
        double maxThrottle = 0.0;
    };

    class Vessel {
    public:
        explicit Vessel(const std::string& name) : m_Name(name), m_RCS(50000.0) {}

        void AddPart(std::shared_ptr<Part> part) {
            m_Parts.push_back(part);
            m_Staging.RebuildStages(m_Parts);
            RecalculateMass();
        }

        void ActivateNextStage() {
            m_Staging.ActivateNextStage();
            RecalculateMass();
        }

        void SetStageThrottle(int stage, double throttle) {
            const double clamped = std::max(0.0, std::min(1.0, throttle));
            for (const auto& part : m_Parts) {
                auto engine = std::dynamic_pointer_cast<EnginePart>(part);
                if (!engine || !engine->IsActive()) {
                    continue;
                }
                if (engine->GetStage() == stage) {
                    engine->SetThrottle(clamped);
                }
            }
        }

        int CountActiveEnginesInStage(int stage) const {
            int count = 0;
            for (const auto& part : m_Parts) {
                auto engine = std::dynamic_pointer_cast<EnginePart>(part);
                if (engine && engine->IsActive() && engine->GetStage() == stage) {
                    ++count;
                }
            }
            return count;
        }

        double GetPropellantRemainingMass(int stage, PropellantType type) const {
            double remaining = 0.0;
            for (const auto& part : m_Parts) {
                auto tank = std::dynamic_pointer_cast<FuelTankPart>(part);
                if (!tank || tank->IsDecoupled() || tank->GetStage() != stage) {
                    continue;
                }
                if (tank->GetPropellantType() != type) {
                    continue;
                }
                remaining += tank->GetCurrentFuel();
            }
            return remaining;
        }

        void RecalculateMass() {
            double totalMass = 0.0;
            for (const auto& part : m_Parts) {
                totalMass += part->GetMass();
            }
            m_PhysicsBody.SetMass(totalMass);
        }

        EngineStatus Update(double dt, double ambientPressure) {
            EngineStatus status;
            if (dt <= 0.0) {
                return status;
            }

            for (auto& part : m_Parts) {
                part->Update(dt);
            }

            for (auto& part : m_Parts) {
                auto engine = std::dynamic_pointer_cast<EnginePart>(part);
                if (!engine || !engine->IsActive() || engine->GetThrottle() <= 0.0) {
                    continue;
                }

                status.maxThrottle = std::max(status.maxThrottle, engine->GetThrottle());

                const double requiredMass = engine->GetCurrentMassFlowRate() * dt;
                if (requiredMass <= 0.0) {
                    continue;
                }

                const double fuelRequired = requiredMass * engine->GetFuelMassFraction();
                const double oxidizerRequired = requiredMass * engine->GetOxidizerMassFraction();

                const double fuelProvided = ConsumePropellantForStage(
                    engine->GetStage(),
                    engine->GetFuelType(),
                    fuelRequired);

                double oxidizerProvided = oxidizerRequired;
                if (engine->GetOxidizerType() != PropellantType::None) {
                    oxidizerProvided = ConsumePropellantForStage(
                        engine->GetStage(),
                        engine->GetOxidizerType(),
                        oxidizerRequired);
                }

                const double fuelRatio = (fuelRequired > 0.0) ? (fuelProvided / fuelRequired) : 1.0;
                const double oxidizerRatio = (oxidizerRequired > 0.0) ? (oxidizerProvided / oxidizerRequired) : 1.0;
                const double burnRatio = std::max(0.0, std::min(1.0, std::min(fuelRatio, oxidizerRatio)));

                if (burnRatio <= 0.0) {
                    engine->SetActive(false);
                    engine->SetThrottle(0.0);
                    continue;
                }

                status.totalThrust += engine->GetThrust(ambientPressure) * burnRatio;
                status.totalMassFlow += (requiredMass * burnRatio) / dt;
                status.totalFuelFlow += (fuelRequired * burnRatio) / dt;
                status.totalOxidizerFlow += (oxidizerRequired * burnRatio) / dt;
                ++status.activeEngines;
            }

            RecalculateMass();
            m_PhysicsBody.AddForce(m_PhysicsBody.GetOrientation() * status.totalThrust);
            return status;
        }

        PhysicsBody& GetPhysicsBody() { return m_PhysicsBody; }
        const PhysicsBody& GetPhysicsBody() const { return m_PhysicsBody; }

        const std::string& GetName() const { return m_Name; }
        const std::vector<std::shared_ptr<Part>>& GetParts() const { return m_Parts; }
        StagingSystem& GetStaging() { return m_Staging; }
        RCS& GetRCS() { return m_RCS; }

    private:
        double ConsumePropellantForStage(int stage, PropellantType type, double requiredMass) {
            if (requiredMass <= 0.0 || type == PropellantType::None) {
                return 0.0;
            }

            double remaining = requiredMass;
            for (auto& part : m_Parts) {
                auto tank = std::dynamic_pointer_cast<FuelTankPart>(part);
                if (!tank || tank->IsDecoupled() || tank->GetStage() != stage) {
                    continue;
                }
                if (tank->GetPropellantType() != type) {
                    continue;
                }

                const double available = tank->GetCurrentFuel();
                if (available <= 0.0) {
                    continue;
                }

                const double toConsume = std::min(remaining, available);
                tank->ConsumeFuel(toConsume);
                remaining -= toConsume;
                if (remaining <= 1e-6) {
                    break;
                }
            }

            return requiredMass - remaining;
        }

    private:
        std::string m_Name;
        std::vector<std::shared_ptr<Part>> m_Parts;
        PhysicsBody m_PhysicsBody;
        StagingSystem m_Staging;
        RCS m_RCS;
    };
}
