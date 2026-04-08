#pragma once
#include <vector>
#include <string>
#include "MissionData.h"

namespace DeepSpace {

class MissionScript;
class Vessel;

struct MissionEventDef {
    std::string name;
    std::string description;
    std::vector<TriggerCondition> triggers;
    std::vector<Command> commands;
    bool triggered = false;
};

struct ExitCondition {
    std::string name;
    std::string type;
    double threshold = 0.0;
    int stage = -1;
    bool mandatory = false;
};

struct TargetOrbit {
    double apoapsis_km = 185.0;
    double periapsis_km = 180.0;
    double inclination_deg = 28.5;
};

struct MissionScript {
    std::string name;
    std::string description;
    TargetOrbit targetOrbit;
    
    std::vector<MissionEvent> events;
    std::vector<ExitCondition> successConditions;
    std::vector<ExitCondition> failureConditions;
    std::vector<ExitCondition> abortConditions;
    
    double maxDuration_s = 7200.0;
    bool autoMode = true;
};

class EventTriggerSystem {
public:
    EventTriggerSystem() = default;
    
    void LoadScript(const MissionScript& script);
    void Reset();
    
    void Update(const class Vessel& vessel, double missionTime, double altitude, 
                double velocity, double maxQ, double damage);
    
    std::vector<Command> CheckTriggers(const class Vessel& vessel, double missionTime,
                                       double altitude, double velocity, double maxQ, double damage);
    
    bool CheckCondition(const TriggerCondition& cond, const class Vessel& vessel,
                       double missionTime, double altitude, double velocity,
                       double maxQ, double damage) const;
    
    const std::vector<MissionEvent>& GetPendingEvents() const { return m_Events; }
    const std::vector<MissionEvent>& GetTriggeredEvents() const { return m_TriggeredEvents; }
    
private:
    const std::vector<TriggerCondition>& GetEventTriggers(const std::string& eventName) const;
    const std::vector<Command>& GetEventCommands(const std::string& eventName) const;
    
    MissionScript m_Script;
    std::vector<MissionEvent> m_Events;
    std::vector<MissionEvent> m_TriggeredEvents;
};

} // namespace DeepSpace
