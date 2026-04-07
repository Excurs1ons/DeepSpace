#include "DamageSystem.h"
#include <algorithm>

namespace DeepSpace {

DamageSystem::DamageSystem() {
    std::random_device rd;
    m_RandomEngine.seed(rd());
    InitializeDefaultCascades();
}

void DamageSystem::InitializeDefaultCascades() {
    m_Cascades = {
        {DamageType::STRUCTURAL, DamageType::LIFESUPPORT, 0.5, 0.5, 0.3},
        {DamageType::PROPULSION, DamageType::STRUCTURAL, 0.3, 1.0, 0.2},
        {DamageType::PROPULSION, DamageType::LIFESUPPORT, 0.5, 0.8, 0.4},
        {DamageType::LIFESUPPORT, DamageType::STRUCTURAL, 0.6, 1.5, 0.3},
        {DamageType::TPS, DamageType::LIFESUPPORT, 0.8, 2.0, 0.5}
    };
}

void DamageSystem::RegisterCascade(CascadeEffect effect) {
    m_Cascades.push_back(effect);
}

double DamageSystem::CalculateCascadeProbability(const CascadeEffect& effect, double sourceDamage) const {
    double damageFactor = std::clamp(sourceDamage, 0.0, 1.0);
    double prob = effect.probability * damageFactor * effect.magnitude;
    return std::clamp(prob, 0.0, 1.0);
}

void DamageSystem::ApplyCascade(const CascadeEffect& effect, Vessel& vessel) {
    double currentTargetDamage = GetDamageFromVessel(effect.targetType, vessel);
    double newDamage = std::clamp(currentTargetDamage + effect.magnitude, 0.0, 1.0);
    SetDamageInVessel(effect.targetType, newDamage, vessel);
    
    switch (effect.targetType) {
        case DamageType::TPS: m_TPSDamage = newDamage; break;
        case DamageType::STRUCTURAL: m_StructuralDamage = newDamage; break;
        case DamageType::PROPULSION: m_PropulsionDamage = newDamage; break;
        case DamageType::LIFESUPPORT: m_LifeSupportDamage = newDamage; break;
    }
}

void DamageSystem::UpdatePendingCascades(double dt, Vessel& vessel) {
    m_TimeAccumulator += dt;
    
    auto it = m_PendingCascades.begin();
    while (it != m_PendingCascades.end()) {
        it->delay -= dt;
        if (it->delay <= 0.0) {
            ApplyCascade(*it, vessel);
            it = m_PendingCascades.erase(it);
        } else {
            ++it;
        }
    }
}

void DamageSystem::Update(double dt, Vessel& vessel) {
    UpdatePendingCascades(dt, vessel);

    for (const auto& cascade : m_Cascades) {
        double sourceDamage = GetDamageFromVessel(cascade.sourceType, vessel);
        
        if (sourceDamage > 0.1) {
            double probability = CalculateCascadeProbability(cascade, sourceDamage);
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            
            if (dist(m_RandomEngine) < probability) {
                CascadeEffect pendingCascade = cascade;
                pendingCascade.delay = 0.1 + (cascade.delay * (1.0 - sourceDamage));
                m_PendingCascades.push_back(pendingCascade);
            }
        }
    }
}

void DamageSystem::TriggerDamage(DamageType type, double amount, Vessel& vessel) {
    double currentDamage = GetDamageFromVessel(type, vessel);
    double newDamage = std::clamp(currentDamage + amount, 0.0, 1.0);
    SetDamageInVessel(type, newDamage, vessel);
    
    switch (type) {
        case DamageType::TPS: m_TPSDamage = newDamage; break;
        case DamageType::STRUCTURAL: m_StructuralDamage = newDamage; break;
        case DamageType::PROPULSION: m_PropulsionDamage = newDamage; break;
        case DamageType::LIFESUPPORT: m_LifeSupportDamage = newDamage; break;
    }
}

double DamageSystem::GetVesselHealth() const {
    static const std::unordered_map<DamageType, double> weights = {
        {DamageType::LIFESUPPORT, 0.4},
        {DamageType::PROPULSION, 0.3},
        {DamageType::STRUCTURAL, 0.2},
        {DamageType::TPS, 0.1}
    };
    
    double healthSum = 0.0;
    double weightSum = 0.0;
    
    double tpsHealth = 1.0 - m_TPSDamage;
    double structHealth = 1.0 - m_StructuralDamage;
    double propHealth = 1.0 - m_PropulsionDamage;
    double lifeHealth = 1.0 - m_LifeSupportDamage;
    
    healthSum = tpsHealth * 0.1 + structHealth * 0.2 + propHealth * 0.3 + lifeHealth * 0.4;
    weightSum = 1.0;
    
    return weightSum > 0.0 ? healthSum / weightSum : 1.0;
}

double DamageSystem::GetDamageLevel(DamageType type) const {
    switch (type) {
        case DamageType::TPS: return m_TPSDamage;
        case DamageType::STRUCTURAL: return m_StructuralDamage;
        case DamageType::PROPULSION: return m_PropulsionDamage;
        case DamageType::LIFESUPPORT: return m_LifeSupportDamage;
        default: return 0.0;
    }
}

std::vector<DamageType> DamageSystem::GetCriticalSystems() const {
    std::vector<DamageType> critical;
    
    if (GetDamageFromVessel(DamageType::LIFESUPPORT, Vessel("")) > DAMAGE_THRESHOLD_CRITICAL) {
        critical.push_back(DamageType::LIFESUPPORT);
    }
    if (GetDamageFromVessel(DamageType::PROPULSION, Vessel("")) > DAMAGE_THRESHOLD_CRITICAL) {
        critical.push_back(DamageType::PROPULSION);
    }
    if (GetDamageFromVessel(DamageType::STRUCTURAL, Vessel("")) > DAMAGE_THRESHOLD_CRITICAL) {
        critical.push_back(DamageType::STRUCTURAL);
    }
    if (GetDamageFromVessel(DamageType::TPS, Vessel("")) > DAMAGE_THRESHOLD_CRITICAL) {
        critical.push_back(DamageType::TPS);
    }
    
    return critical;
}

void DamageSystem::ApplyRepair(DamageType type, double amount) {
    Vessel dummy("");
    double currentDamage = GetDamageFromVessel(type, dummy);
    double newDamage = std::clamp(currentDamage - amount, 0.0, 1.0);
    SetDamageInVessel(type, newDamage, dummy);
}

double DamageSystem::GetDamageFromVessel(DamageType type, const Vessel& vessel) const {
    switch (type) {
        case DamageType::TPS: return vessel.m_TPSDamage;
        case DamageType::STRUCTURAL: return vessel.m_StructuralDamage;
        case DamageType::PROPULSION: return vessel.m_PropulsionDamage;
        case DamageType::LIFESUPPORT: return vessel.m_LifeSupportDamage;
        default: return 0.0;
    }
}

void DamageSystem::SetDamageInVessel(DamageType type, double value, Vessel& vessel) const {
    switch (type) {
        case DamageType::TPS: vessel.m_TPSDamage = value; break;
        case DamageType::STRUCTURAL: vessel.m_StructuralDamage = value; break;
        case DamageType::PROPULSION: vessel.m_PropulsionDamage = value; break;
        case DamageType::LIFESUPPORT: vessel.m_LifeSupportDamage = value; break;
    }
}

bool DamageSystem::IsSystemDestroyed(DamageType type) const {
    return GetDamageFromVessel(type, Vessel("")) >= DAMAGE_THRESHOLD_DESTROYED;
}

bool DamageSystem::IsSystemCritical(DamageType type) const {
    double damage = GetDamageFromVessel(type, Vessel(""));
    return damage > DAMAGE_THRESHOLD_CRITICAL && damage < DAMAGE_THRESHOLD_DESTROYED;
}

} // namespace DeepSpace