#pragma once
#include "MissionData.h"
#include "MissionScript.h"
#include "../vessel/Vessel.h"
#include "../environment/Planet.h"
#include "../physics/OrbitalElements.h"
#include <fstream>
#include <iostream>
#include <iomanip>

namespace DeepSpace {

class MissionControl {
public:
    MissionControl(Vessel& vessel, Planet& earth)
        : m_Vessel(vessel), m_Earth(earth) {}
    
    void LoadMission(const MissionScript& script) {
        m_Script = script;
        m_TriggerSystem.LoadScript(script);
        Reset();
        MOCK_INFO("Mission loaded: %s", script.name.c_str());
    }
    
    void Reset() {
        m_MissionTime = 0.0;
        m_CurrentPhase = MissionPhase::PRE_LAUNCH;
        m_Outcome = MissionOutcome::IN_PROGRESS;
        m_TriggerSystem.Reset();
        m_Summary.missionName = m_Script.name;
        m_Summary.targetOrbit.apoapsis_m = m_Script.targetOrbit.apoapsis_km * 1000.0;
        m_Summary.targetOrbit.periapsis_m = m_Script.targetOrbit.periapsis_km * 1000.0;
        m_TriggeredEvents.clear();
        m_StartTime = std::chrono::system_clock::now();
    }
    
    void Update(double dt) {
        UpdateTelemetry();
    }
    
    void Update(double dt, const EngineStatus& engineStatus) {
        if (m_Outcome != MissionOutcome::IN_PROGRESS) return;
        
        m_MissionTime += dt;
        
        m_LastEngineStatus = engineStatus;
        
        UpdatePhase();
        
        auto commands = m_TriggerSystem.CheckTriggers(
            m_Vessel, m_MissionTime,
            GetCurrentAltitude(),
            GetCurrentVelocity(),
            m_MaxQ,
            GetTotalDamage()
        );
        
        ExecuteCommands(commands);
        
        UpdateTelemetry();
        
        CheckExitConditions();
        
        LogPhaseChange();
    }
    
    double GetMissionTime() const { return m_MissionTime; }
    MissionPhase GetCurrentPhase() const { return m_CurrentPhase; }
    MissionOutcome GetOutcome() const { return m_Outcome; }
    const TelemetryData& GetTelemetry() const { return m_Telemetry; }
    const MissionSummary& GetSummary() const { return m_Summary; }
    
    void TriggerEvent(const std::string& name, const std::string& desc) {
        MissionEvent evt;
        evt.time = m_MissionTime;
        evt.name = name;
        evt.description = desc;
        evt.phase = m_CurrentPhase;
        
        m_TriggeredEvents.push_back(evt);
        m_Summary.allEvents.push_back(evt);
        
        MOCK_INFO("[EVENT] T=%.1fs %s: %s", m_MissionTime, name.c_str(), desc.c_str());
    }
    
    void SetPhase(MissionPhase phase) {
        if (phase != m_CurrentPhase) {
            m_CurrentPhase = phase;
            MOCK_INFO("[PHASE] T=%.1fs -> %s", m_MissionTime, MissionPhaseToString(phase));
        }
    }
    
    void AbortMission(const std::string& reason) {
        m_Outcome = MissionOutcome::ABORT;
        TriggerEvent("ABORT", reason);
        FinalizeSummary();
        MOCK_WARN("[ABORT] Mission aborted: %s", reason.c_str());
    }
    
    void ExportCSV(const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            MOCK_ERROR("Failed to open CSV file: %s", filename.c_str());
            return;
        }
        
        file << "time,phase,altitude_m,velocity_mps,mach,q_pa,apoapsis_m,periapsis_m,mass_kg,thrust_N,throttle_pct,"
              << "maxQ_pa,damage_total,damage_tps,damage_struct,damage_prop,damage_life,survival_pct\n";
        
        for (const auto& row : m_TelemetryLog) {
            file << std::fixed << std::setprecision(2)
                 << row.missionTime << ","
                 << MissionPhaseToString(row.phase) << ","
                 << row.altitude_m << ","
                 << row.velocity_mps << ","
                 << row.mach << ","
                 << row.dynamicPressure_pa << ","
                 << row.orbit.apoapsis_m << ","
                 << row.orbit.periapsis_m << ","
                 << row.totalMass_kg << ","
                 << row.thrust_N << ","
                 << row.throttle_pct << ","
                 << row.maxQ_pa << ","
                 << row.damageTotal << ","
                 << row.damageTPS << ","
                 << row.damageStructural << ","
                 << row.damagePropulsion << ","
                 << row.damageLifeSupport << ","
                 << row.survivalProbability * 100 << "\n";
        }
        
        file.close();
        MOCK_INFO("Telemetry exported to: %s", filename.c_str());
    }
    
    void ExportSummary(const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) return;
        
        file << "{\n";
        file << "  \"mission\": \"" << m_Summary.missionName << "\",\n";
        file << "  \"duration_s\": " << m_Summary.duration_s << ",\n";
        file << "  \"outcome\": \"" << OutcomeToString(m_Outcome) << "\",\n";
        file << "  \"final_orbit\": {\n";
        file << "    \"apoapsis_km\": " << m_Summary.finalOrbit.apoapsis_m / 1000.0 << ",\n";
        file << "    \"periapsis_km\": " << m_Summary.finalOrbit.periapsis_m / 1000.0 << "\n";
        file << "  },\n";
        file << "  \"target_orbit\": {\n";
        file << "    \"apoapsis_km\": " << m_Summary.targetOrbit.apoapsis_m / 1000.0 << ",\n";
        file << "    \"periapsis_km\": " << m_Summary.targetOrbit.periapsis_m / 1000.0 << "\n";
        file << "  },\n";
        file << "  \"maxQ\": {\n";
        file << "    \"value_pa\": " << m_Summary.maxQ_pa << ",\n";
        file << "    \"altitude_m\": " << m_Summary.maxQAltitude_m << ",\n";
        file << "    \"time_s\": " << m_Summary.maxQTime_s << "\n";
        file << "  },\n";
        file << "  \"events\": [\n";
        for (size_t i = 0; i < m_TriggeredEvents.size(); ++i) {
            const auto& e = m_TriggeredEvents[i];
            file << "    {\"time\":" << e.time << ",\"name\":\"" << e.name << "\",\"desc\":\"" << e.description << "\"}";
            if (i < m_TriggeredEvents.size() - 1) file << ",";
            file << "\n";
        }
        file << "  ]\n";
        file << "}\n";
        
        file.close();
        MOCK_INFO("Summary exported to: %s", filename.c_str());
    }

private:
    void UpdatePhase() {
        const auto& body = m_Vessel.GetPhysicsBody();
        const double altitude = m_Earth.GetAltitude(body.GetPosition());
        const double velocity = body.GetVelocity().Length();
        const double orbitalVel = OrbitalMechanics::CircularOrbitVelocity(altitude, m_Earth);
        
        if (m_CurrentPhase == MissionPhase::PRE_LAUNCH && m_MissionTime >= 0) {
            SetPhase(MissionPhase::LAUNCH);
        }
        else if (m_CurrentPhase == MissionPhase::LAUNCH && altitude > 1000) {
            SetPhase(MissionPhase::ASCENT);
        }
        else if (m_CurrentPhase == MissionPhase::ASCENT && m_MaxQPassed) {
            SetPhase(MissionPhase::MAX_Q);
        }
        else if (m_CurrentPhase == MissionPhase::MAX_Q && velocity > orbitalVel * 0.95) {
            SetPhase(MissionPhase::ORBIT);
        }
    }
    
    void UpdateTelemetry() {
        const auto& body = m_Vessel.GetPhysicsBody();
        const Vec3d pos = body.GetPosition();
        const Vec3d vel = body.GetVelocity();
        const double altitude = m_Earth.GetAltitude(pos);
        const double velocity = vel.Length();
        const double density = m_Earth.GetAtmosphere().GetDensity(altitude);
        
        TelemetryData data;
        data.missionTime = m_MissionTime;
        data.phase = m_CurrentPhase;
        data.altitude_m = altitude;
        data.velocity_mps = velocity;
        data.mach = velocity / 340.0;
        data.dynamicPressure_pa = 0.5 * density * velocity * velocity;
        data.totalMass_kg = body.GetMass();
        data.damageTotal = m_Vessel.GetTotalDamage();
        
        data.thrust_N = m_LastEngineStatus.totalThrust;
        data.throttle_pct = m_LastEngineStatus.maxThrottle * 100.0;
        data.massFlow_kg_s = m_LastEngineStatus.totalMassFlow;
        data.fuelFlow_kg_s = m_LastEngineStatus.totalFuelFlow;
        data.oxidizerFlow_kg_s = m_LastEngineStatus.totalOxidizerFlow;
        
        if (data.dynamicPressure_pa > m_MaxQ) {
            m_MaxQ = data.dynamicPressure_pa;
            m_MaxQAltitude = altitude;
            m_MaxQTime = m_MissionTime;
            m_Summary.maxQ_pa = m_MaxQ;
            m_Summary.maxQAltitude_m = m_MaxQAltitude;
            m_Summary.maxQTime_s = m_MaxQTime;
        }
        
        data.maxQ_pa = m_MaxQ;
        
        if (m_MaxQ > 1000 && !m_MaxQPassed) {
            m_MaxQPassed = true;
            TriggerEvent("MaxQ_Pass", "Dynamic pressure peak passed");
        }
        
        auto orbit = OrbitalMechanics::CalculateElements(pos, vel, m_Earth);
        data.orbit.apoapsis_m = orbit.apoapsis;
        data.orbit.periapsis_m = orbit.periapsis;
        data.orbit.isBound = orbit.isBound;
        
        m_Telemetry = data;
        
        static double lastLogTime = 0;
        if (m_MissionTime - lastLogTime >= 2.0) {
            m_TelemetryLog.push_back(data);
            lastLogTime = m_MissionTime;
        }
    }
    
    void ExecuteCommands(const std::vector<Command>& commands) {
        for (const auto& cmd : commands) {
            switch (cmd.type) {
                case CommandType::STAGE_SEPARATION:
                    m_Vessel.ActivateNextStage();
                    TriggerEvent("Stage_Separation", "Stage " + std::to_string(cmd.stage) + " separated");
                    m_Summary.stagingEvents.push_back(m_TriggeredEvents.back());
                    break;
                case CommandType::SET_THROTTLE:
                    m_Vessel.SetStageThrottle(cmd.stage, cmd.value);
                    break;
                case CommandType::LOG_MESSAGE:
                    MOCK_INFO("[CMD] %s", cmd.message.c_str());
                    break;
                case CommandType::ABORT_MISSION:
                    AbortMission(cmd.message);
                    break;
                case CommandType::TRIGGER_DAMAGE:
                    TriggerEvent("Damage_Applied", cmd.message);
                    break;
            }
        }
    }
    
    void CheckExitConditions() {
        if (m_MissionTime > m_Script.maxDuration_s) {
            m_Outcome = MissionOutcome::TIMEOUT;
            TriggerEvent("Timeout", "Mission exceeded maximum duration");
            FinalizeSummary();
        }
        
        const auto& body = m_Vessel.GetPhysicsBody();
        if (m_Earth.GetAltitude(body.GetPosition()) < 0 && m_MissionTime > 10) {
            m_Outcome = MissionOutcome::FAILURE;
            TriggerEvent("Crash", "Vehicle impacted surface");
            FinalizeSummary();
        }
        
        if (m_CurrentPhase == MissionPhase::ORBIT && m_MissionTime > 100) {
            auto orbit = OrbitalMechanics::CalculateElements(
                body.GetPosition(), body.GetVelocity(), m_Earth);
            
            if (orbit.isBound) {
                double apError = std::abs(orbit.apoapsis - m_Script.targetOrbit.apoapsis_km * 1000);
                double peError = std::abs(orbit.periapsis - m_Script.targetOrbit.periapsis_km * 1000);
                
                if (apError < 10000 && peError < 10000) {
                    m_Outcome = MissionOutcome::SUCCESS;
                    m_Summary.finalOrbit = m_Telemetry.orbit;
                    TriggerEvent("Mission_Complete", "Target orbit achieved");
                    FinalizeSummary();
                }
            }
        }
    }
    
    void FinalizeSummary() {
        m_Summary.endTime = std::to_string(std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::now()));
        m_Summary.duration_s = m_MissionTime;
        m_Summary.outcome = m_Outcome;
    }
    
    void LogPhaseChange() {
        static MissionPhase lastPhase = MissionPhase::PRE_LAUNCH;
        if (m_CurrentPhase != lastPhase) {
            lastPhase = m_CurrentPhase;
            MOCK_TRACE("[PHASE] %s", MissionPhaseToString(m_CurrentPhase));
        }
    }
    
    double GetCurrentAltitude() const {
        return m_Earth.GetAltitude(m_Vessel.GetPhysicsBody().GetPosition());
    }
    
    double GetCurrentVelocity() const {
        return m_Vessel.GetPhysicsBody().GetVelocity().Length();
    }
    
    double GetTotalDamage() const {
        return m_Vessel.GetTotalDamage();
    }
    
    const char* OutcomeToString(MissionOutcome outcome) {
        switch (outcome) {
            case MissionOutcome::IN_PROGRESS: return "IN_PROGRESS";
            case MissionOutcome::SUCCESS: return "SUCCESS";
            case MissionOutcome::FAILURE: return "FAILURE";
            case MissionOutcome::ABORT: return "ABORT";
            case MissionOutcome::TIMEOUT: return "TIMEOUT";
            default: return "UNKNOWN";
        }
    }
    
    Vessel& m_Vessel;
    Planet& m_Earth;
    
    MissionScript m_Script;
    EventTriggerSystem m_TriggerSystem;
    
    double m_MissionTime = 0.0;
    MissionPhase m_CurrentPhase = MissionPhase::PRE_LAUNCH;
    MissionOutcome m_Outcome = MissionOutcome::IN_PROGRESS;
    
    double m_MaxQ = 0.0;
    double m_MaxQAltitude = 0.0;
    double m_MaxQTime = 0.0;
    bool m_MaxQPassed = false;
    
    TelemetryData m_Telemetry;
    std::vector<TelemetryData> m_TelemetryLog;
    std::vector<MissionEvent> m_TriggeredEvents;
    MissionSummary m_Summary;
    EngineStatus m_LastEngineStatus;
    
    std::chrono::system_clock::time_point m_StartTime;
};

} // namespace DeepSpace
