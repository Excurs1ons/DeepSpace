#include "MissionScript.h"

namespace DeepSpace {

void EventTriggerSystem::LoadScript(const MissionScript& script) {
    m_Script = script;
    m_Events.clear();
    m_TriggeredEvents.clear();
    
    for (const auto& eventDef : m_Script.events) {
        MissionEvent evt;
        evt.name = eventDef.name;
        evt.description = "";
        evt.phase = MissionPhase::PRE_LAUNCH;
        evt.time = 0.0;
        m_Events.push_back(evt);
    }
}

void EventTriggerSystem::Reset() {
    for (auto& evt : m_Events) {
        evt.triggered = false;
    }
    m_TriggeredEvents.clear();
}

void EventTriggerSystem::Update(const Vessel& vessel, double missionTime, double altitude, 
                                 double velocity, double maxQ, double damage) {
    CheckTriggers(vessel, missionTime, altitude, velocity, maxQ, damage);
}

std::vector<Command> EventTriggerSystem::CheckTriggers(const Vessel& vessel, double missionTime,
                                                       double altitude, double velocity, 
                                                       double maxQ, double damage) {
    std::vector<Command> triggeredCommands;
    
    for (auto& evt : m_Events) {
        if (evt.triggered) continue;
        
        bool allTriggersMet = true;
        for (const auto& cond : GetEventTriggers(evt.name)) {
            if (!CheckCondition(cond, vessel, missionTime, altitude, velocity, maxQ, damage)) {
                allTriggersMet = false;
                break;
            }
        }
        
        if (allTriggersMet) {
            evt.triggered = true;
            evt.time = missionTime;
            m_TriggeredEvents.push_back(evt);
            
            for (const auto& cmd : GetEventCommands(evt.name)) {
                triggeredCommands.push_back(cmd);
            }
        }
    }
    
    return triggeredCommands;
}

bool EventTriggerSystem::CheckCondition(const TriggerCondition& cond, const Vessel& vessel,
                                         double missionTime, double altitude, double velocity,
                                         double maxQ, double damage) const {
    if (cond.once && cond.triggered) {
        return false;
    }
    
    switch (cond.type) {
        case TriggerType::TIME_ELAPSED:
            return missionTime >= cond.value;
            
        case TriggerType::ALTITUDE_ABOVE:
            return altitude > cond.value;
            
        case TriggerType::ALTITUDE_BELOW:
            return altitude < cond.value;
            
        case TriggerType::VELOCITY_ABOVE:
            return velocity > cond.value;
            
        case TriggerType::VELOCITY_BELOW:
            return velocity < cond.value;
            
        case TriggerType::MAXQ_PASSED:
            return maxQ > cond.value;
            
        case TriggerType::DAMAGE_EXCEEDED:
            return damage >= cond.value;
            
        default:
            return false;
    }
}

const std::vector<TriggerCondition>& EventTriggerSystem::GetEventTriggers(const std::string& eventName) const {
    for (const auto& evt : m_Script.events) {
        if (evt.name == eventName) {
            return evt.triggers;
        }
    }
    static std::vector<TriggerCondition> empty;
    return empty;
}

const std::vector<Command>& EventTriggerSystem::GetEventCommands(const std::string& eventName) const {
    for (const auto& evt : m_Script.events) {
        if (evt.name == eventName) {
            return evt.commands;
        }
    }
    static std::vector<Command> empty;
    return empty;
}

} // namespace DeepSpace
