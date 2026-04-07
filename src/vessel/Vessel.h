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
        : m_Name(name), m_RCS(100.0), m_CurrentStage(0) {}

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

    void ActivateNextStage() {
        m_CurrentStage++;
        for (auto& part : m_Parts) {
            if (part->GetStage() == m_CurrentStage && !part->IsDecoupled()) {
                part->SetActive(true);
            }
        }
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

        double totalMass = m_Body.GetMass();
        Vec3d totalThrust{0.0, 0.0, 0.0};

        for (auto& part : m_Parts) {
            if (!part->IsActive() || part->IsDecoupled()) continue;

            totalMass += part->GetMass();

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
        double mass = 0.0;
        for (const auto& part : m_Parts) {
            mass += part->GetMass();
        }
        return mass;
    }

private:
    std::string m_Name;
    PhysicsBody m_Body;
    RCS m_RCS;
    std::vector<std::shared_ptr<Part>> m_Parts;
    int m_CurrentStage;
    
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
