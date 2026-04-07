#pragma once
#include "../vessel/Vessel.h"
#include <vector>
#include <unordered_map>
#include <random>

namespace DeepSpace {

enum class DamageType {
    TPS,
    STRUCTURAL,
    PROPULSION,
    LIFESUPPORT
};

struct CascadeEffect {
    DamageType sourceType;
    DamageType targetType;
    double probability;
    double delay;
    double magnitude;
};

class DamageSystem {
public:
    DamageSystem();

    void RegisterCascade(CascadeEffect effect);
    void Update(double dt, Vessel& vessel);
    void TriggerDamage(DamageType type, double amount, Vessel& vessel);
    double GetVesselHealth() const;
    std::vector<DamageType> GetCriticalSystems() const;
    void ApplyRepair(DamageType type, double amount);

    double GetDamageLevel(DamageType type) const;
    bool IsSystemDestroyed(DamageType type) const;
    bool IsSystemCritical(DamageType type) const;

private:
    void InitializeDefaultCascades();
    double CalculateCascadeProbability(const CascadeEffect& effect, double sourceDamage) const;
    void ApplyCascade(const CascadeEffect& effect, Vessel& vessel);
    void UpdatePendingCascades(double dt, Vessel& vessel);

    double GetDamageFromVessel(DamageType type, const Vessel& vessel) const;
    void SetDamageInVessel(DamageType type, double value, Vessel& vessel) const;

    std::vector<CascadeEffect> m_Cascades;
    std::vector<CascadeEffect> m_PendingCascades;
    
    double m_TimeAccumulator = 0.0;
    std::mt19937 m_RandomEngine;
    
    double m_TPSDamage = 0.0;
    double m_StructuralDamage = 0.0;
    double m_PropulsionDamage = 0.0;
    double m_LifeSupportDamage = 0.0;

    static constexpr double DAMAGE_THRESHOLD_CRITICAL = 0.8;
    static constexpr double DAMAGE_THRESHOLD_DESTROYED = 1.0;
};

} // namespace DeepSpace