#pragma once
#include "Part.h"
#include "RCS.h"
#include "../physics/PhysicsBody.h"
#include "../environment/DamageComponent.h"
#include <map>
#include <vector>
#include <memory>

namespace DeepSpace {

struct EngineStatus {
    int activeEngines = 0;
    double totalThrust = 0.0;
    double maxThrottle = 0.0;
    double totalMassFlow = 0.0;
    double totalFuelFlow = 0.0;
    double totalOxidizerFlow = 0.0;
};

class TPSDamage;
class StructuralDamage;
class PropulsionDamage;
class LifeSupportDamage;

class Vessel {
public:
    explicit Vessel(const std::string& name)
        : m_Name(name), m_RCS(100.0), m_CurrentStage(-1), m_HighestStage(-1) {}

    const std::string& GetName() const { return m_Name; }

    PhysicsBody& GetPhysicsBody() { return m_Body; }
    const PhysicsBody& GetPhysicsBody() const { return m_Body; }

    RCS& GetRCS() { return m_RCS; }
    const RCS& GetRCS() const { return m_RCS; }

    double GetTotalDamage() const;
    void ApplyDamage(double amount, const Vec3d& location = {0, 0, 0});
    void UpdateWithDamage(double dt, double ambientPressure);

    void AddPart(std::shared_ptr<Part> part) {
        m_Parts.push_back(part);
    }
    
    const std::vector<std::shared_ptr<Part>>& GetParts() const { return m_Parts; }

    void ActivateNextStage() {
        if (m_HighestStage < 0) {
            m_HighestStage = FindHighestStage();
            m_CurrentStage = -1;
        }
        
        if (m_CurrentStage >= 0) {
            for (auto& part : m_Parts) {
                if (part->GetStage() == m_CurrentStage && !part->IsDecoupled()) {
                    part->SetDecoupled(true);
                }
            }
        }
        
        m_CurrentStage++;
        if (m_CurrentStage <= m_HighestStage) {
            for (auto& part : m_Parts) {
                if (part->GetStage() == m_CurrentStage && !part->IsDecoupled()) {
                    part->SetActive(true);
                    auto* engine = dynamic_cast<EnginePart*>(part.get());
                    if (engine) {
                        engine->SetThrottle(1.0);
                    }
                }
            }
        }
    }
    
    int FindHighestStage() const {
        int maxStage = -1;
        for (const auto& part : m_Parts) {
            maxStage = std::max(maxStage, part->GetStage());
        }
        return maxStage;
    }

    void SetStageThrottle(int stage, double throttle) {
        for (auto& part : m_Parts) {
            if (part->GetStage() == stage && part->IsActive()) {
                auto* engine = dynamic_cast<EnginePart*>(part.get());
                if (engine) {
                    engine->SetThrottle(throttle);
                }
            }
        }
    }

        EngineStatus Update(double dt, double ambientPressure) {
        EngineStatus status;

        if (dt <= 0.0) return status;

        for (auto& part : m_Parts) {
            if (!part->IsActive() || part->IsDecoupled()) continue;

            auto* engine = dynamic_cast<EnginePart*>(part.get());
            if (engine && engine->GetThrottle() > 0.0) {
                const double mdot = engine->GetCurrentMassFlowRate();
                const double fuelToConsume = mdot * engine->GetFuelMassFraction() * dt;
                const double oxToConsume = mdot * engine->GetOxidizerMassFraction() * dt;
                
                bool fuelConsumed = true;
                bool oxConsumed = true;
                
                for (auto& p : m_Parts) {
                    if (p->GetStage() != engine->GetStage() || p->IsDecoupled()) continue;
                    auto* tank = dynamic_cast<FuelTankPart*>(p.get());
                    if (tank) {
                        if (tank->GetPropellantType() == engine->GetFuelType() && fuelToConsume > 0) {
                            if (!tank->ConsumeFuel(fuelToConsume)) fuelConsumed = false;
                        }
                        if (tank->GetPropellantType() == engine->GetOxidizerType() && oxToConsume > 0) {
                            if (!tank->ConsumeFuel(oxToConsume)) oxConsumed = false;
                        }
                    }
                }
                if (!fuelConsumed || !oxConsumed) {
                    engine->SetActive(false);
                }
            }
        }

        double totalMass = 0.0;
        for (const auto& part : m_Parts) {
            if (!part->IsDecoupled()) {
                totalMass += part->GetMass();
            }
        }
        
        Vec3d totalThrust{0.0, 0.0, 0.0};

        for (auto& part : m_Parts) {
            if (!part->IsActive() || part->IsDecoupled()) continue;

            auto* engine = dynamic_cast<EnginePart*>(part.get());
            if (engine && engine->GetThrottle() > 0.0) {
                const double thrust = engine->GetThrust(ambientPressure);
                const Vec3d orientation = m_Body.GetOrientationVec3();
                totalThrust += orientation * thrust;
                status.totalThrust += thrust;
                status.activeEngines++;
                status.maxThrottle = std::max(status.maxThrottle, engine->GetThrottle());

                const double mdot = engine->GetCurrentMassFlowRate();
                status.totalMassFlow += mdot;
                status.totalFuelFlow += mdot * engine->GetFuelMassFraction();
                status.totalOxidizerFlow += mdot * engine->GetOxidizerMassFraction();
            }
        }

        m_Body.AddForce(totalThrust);
        m_Body.SetMass(totalMass);
        
        return status;
    }

    double GetPropellantRemainingMass(int stage, PropellantType type) const {
        double total = 0.0;
        for (const auto& part : m_Parts) {
            if (part->GetStage() == stage && !part->IsDecoupled()) {
                auto* tank = dynamic_cast<const FuelTankPart*>(part.get());
                if (tank && tank->GetPropellantType() == type) {
                    total += tank->GetCurrentFuel();
                }
            }
        }
        return total;
    }

    double GetMass() const {
        return m_Body.GetMass();
    }
    
    void RecalculateMass() {
        double totalMass = 0.0;
        for (const auto& part : m_Parts) {
            if (!part->IsDecoupled()) {
                totalMass += part->GetMass();
            }
        }
        m_Body.SetMass(totalMass);
    }
    std::string m_Name;
    PhysicsBody m_Body;
    RCS m_RCS;
    std::vector<std::shared_ptr<Part>> m_Parts;
    int m_CurrentStage;
    int m_HighestStage;
    
    double m_TPSDamage = 0.0;
    double m_StructuralDamage = 0.0;
    double m_PropulsionDamage = 0.0;
    double m_LifeSupportDamage = 0.0;
    double m_CabinTemperature = 293.15;
    double m_CabinPressure = 101.325;
    double m_OxygenLevel = 0.209;
    double m_CO2Level = 0.0;

    friend class DamageSystem;
};

}
