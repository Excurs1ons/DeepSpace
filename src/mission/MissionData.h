#pragma once
#include <string>
#include <vector>
#include <chrono>
#include "../engine/MockEngine.h"

namespace DeepSpace {
using Vec3d = Mock::Vec3d;

enum class MissionPhase {
    PRE_LAUNCH,
    LAUNCH,
    ASCENT,
    MAX_Q,
    STAGING,
    COAST,
    CIRCULARIZATION,
    ORBIT,
    TEI,
    TRANSLunar,
    MISSION_EVENTS,
    REENTRY,
    SUCCESS,
    FAILURE,
    ABORT
};

inline const char* MissionPhaseToString(MissionPhase phase) {
    switch (phase) {
        case MissionPhase::PRE_LAUNCH: return "PRE_LAUNCH";
        case MissionPhase::LAUNCH: return "LAUNCH";
        case MissionPhase::ASCENT: return "ASCENT";
        case MissionPhase::MAX_Q: return "MAX_Q";
        case MissionPhase::STAGING: return "STAGING";
        case MissionPhase::COAST: return "COAST";
        case MissionPhase::CIRCULARIZATION: return "CIRCULARIZATION";
        case MissionPhase::ORBIT: return "ORBIT";
        case MissionPhase::TEI: return "TEI";
        case MissionPhase::TRANSLunar: return "TRANSLUNAR";
        case MissionPhase::MISSION_EVENTS: return "MISSION_EVENTS";
        case MissionPhase::REENTRY: return "REENTRY";
        case MissionPhase::SUCCESS: return "SUCCESS";
        case MissionPhase::FAILURE: return "FAILURE";
        case MissionPhase::ABORT: return "ABORT";
        default: return "UNKNOWN";
    }
}

enum class MissionOutcome {
    IN_PROGRESS,
    SUCCESS,
    FAILURE,
    ABORT,
    TIMEOUT
};

enum class TriggerType {
    TIME_ELAPSED,
    ALTITUDE_ABOVE,
    ALTITUDE_BELOW,
    VELOCITY_ABOVE,
    VELOCITY_BELOW,
    PROPELLANT_DEPLETED,
    MAXQ_PASSED,
    APOAPSIS_ABOVE,
    PERIAPSIS_ABOVE,
    APOAPSIS_BELOW,
    PERIAPSIS_BELOW,
    ORBIT_CIRCULARIZED,
    DAMAGE_EXCEEDED,
    STAGE_ACTIVATED,
    ENGINE_CUTOFF,
    MACH_ABOVE,
    MACH_BELOW
};

struct TriggerCondition {
    TriggerType type;
    double value = 0.0;
    int stage = -1;
    bool once = false;
    bool triggered = false;
};

enum class CommandType {
    STAGE_SEPARATION,
    SET_THROTTLE,
    SET_ORIENTATION,
    ENABLE_RCS,
    LOG_MESSAGE,
    CIRCULARIZATION_BURN,
    ABORT_MISSION,
    TRIGGER_DAMAGE,
    WAIT
};

struct Command {
    CommandType type;
    int stage = -1;
    double value = 0.0;
    std::string message;
    std::string orientation;
};

struct MissionEvent {
    double time = 0.0;
    std::string name;
    std::string description;
    MissionPhase phase;
    
    bool triggered = false;
    std::vector<TriggerCondition> triggers;
    std::vector<Command> commands;
};

struct OrbitalState {
    double apoapsis_m = 0.0;
    double periapsis_m = 0.0;
    double inclination_deg = 0.0;
    double period_s = 0.0;
    bool isBound = false;
};

struct TelemetryData {
    double missionTime = 0.0;
    double timestamp = 0.0;
    
    MissionPhase phase = MissionPhase::PRE_LAUNCH;
    
    double altitude_m = 0.0;
    double velocity_mps = 0.0;
    double mach = 0.0;
    double dynamicPressure_pa = 0.0;
    double ambientPressure_pa = 0.0;
    
    Vec3d position;
    Vec3d velocity;
    Vec3d acceleration;
    
    double totalMass_kg = 0.0;
    double thrust_N = 0.0;
    double throttle_pct = 0.0;
    double massFlow_kg_s = 0.0;
    double fuelFlow_kg_s = 0.0;
    double oxidizerFlow_kg_s = 0.0;
    
    double s1LH2_kg = 0.0;
    double s1LOX_kg = 0.0;
    double s0MMH_kg = 0.0;
    double s0NTO_kg = 0.0;
    
    double maxQ_pa = 0.0;
    double maxQAltitude_m = 0.0;
    double maxQTime_s = 0.0;
    
    double damageTotal = 0.0;
    double damageTPS = 0.0;
    double damageStructural = 0.0;
    double damagePropulsion = 0.0;
    double damageLifeSupport = 0.0;
    double survivalProbability = 1.0;
    double vesselHealth = 1.0;
    
    OrbitalState orbit;
    
    int activeEngines = 0;
    int currentStage = 0;
    
    std::vector<MissionEvent> recentEvents;
};

struct MissionSummary {
    std::string missionName;
    std::string startTime;
    std::string endTime;
    double duration_s = 0.0;
    MissionOutcome outcome = MissionOutcome::IN_PROGRESS;
    
    double maxQ_pa = 0.0;
    double maxQAltitude_m = 0.0;
    double maxQTime_s = 0.0;
    
    OrbitalState finalOrbit;
    OrbitalState targetOrbit;
    
    std::vector<MissionEvent> stagingEvents;
    std::vector<MissionEvent> allEvents;
    
    double peakAcceleration_g = 0.0;
    double peakHeatFlux_W_m2 = 0.0;
    double totalHeatLoad_J = 0.0;
};

} // namespace DeepSpace
