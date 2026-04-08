#pragma once
#include "MissionScript.h"

namespace DeepSpace {

class Artemis2Mission {
public:
    static MissionScript CreateDefaultMission() {
        MissionScript script;
        script.name = "Artemis II Automated";
        script.description = "Fully automated Artemis II mission profile";
        script.targetOrbit.apoapsis_km = 185.0;
        script.targetOrbit.periapsis_km = 180.0;
        script.targetOrbit.inclination_deg = 28.5;
        script.maxDuration_s = 7200.0;
        script.autoMode = true;

        {
            MissionEvent evt;
            evt.name = "launch";
            evt.description = "Main core engine ignition and liftoff";
            TriggerCondition trigger;
            trigger.type = TriggerType::TIME_ELAPSED;
            trigger.value = 0.0;
            evt.triggers.push_back(trigger);
            Command cmd;
            cmd.type = CommandType::LOG_MESSAGE;
            cmd.message = "T-0: Main engines at full thrust";
            evt.commands.push_back(cmd);
            script.events.push_back(evt);
        }

        {
            MissionEvent evt;
            evt.name = "maxq_throttle";
            evt.description = "Reduce throttle at max-Q";
            TriggerCondition trigger;
            trigger.type = TriggerType::ALTITUDE_ABOVE;
            trigger.value = 9500.0;
            evt.triggers.push_back(trigger);
            TriggerCondition maxQTrigger;
            maxQTrigger.type = TriggerType::MAXQ_PASSED;
            maxQTrigger.value = 1000.0;
            evt.triggers.push_back(maxQTrigger);
            Command cmd;
            cmd.type = CommandType::SET_THROTTLE;
            cmd.stage = 2;
            cmd.value = 0.72;
            evt.commands.push_back(cmd);
            script.events.push_back(evt);
        }

        {
            MissionEvent evt;
            evt.name = "booster_separation";
            evt.description = "Log booster separation event";
            TriggerCondition trigger;
            trigger.type = TriggerType::STAGE_ACTIVATED;
            trigger.value = 1;
            evt.triggers.push_back(trigger);
            Command cmd;
            cmd.type = CommandType::LOG_MESSAGE;
            cmd.message = "Booster separation confirmed - ICPS active";
            evt.commands.push_back(cmd);
            script.events.push_back(evt);
        }

        {
            MissionEvent evt;
            evt.name = "icps_ignition";
            evt.description = "ICPS upper stage ignition";
            TriggerCondition trigger;
            trigger.type = TriggerType::STAGE_ACTIVATED;
            trigger.value = 2;
            evt.triggers.push_back(trigger);
            Command cmd;
            cmd.type = CommandType::LOG_MESSAGE;
            cmd.message = "Orion propulsion takeover - TLI prep";
            evt.commands.push_back(cmd);
            script.events.push_back(evt);
        }

        {
            MissionEvent evt;
            evt.name = "orbit_insertion";
            evt.description = "Confirm orbit insertion";
            TriggerCondition trigger;
            trigger.type = TriggerType::ALTITUDE_ABOVE;
            trigger.value = 100000.0;
            evt.triggers.push_back(trigger);
            Command cmd;
            cmd.type = CommandType::LOG_MESSAGE;
            cmd.message = "Orbit insertion complete - stable orbit achieved";
            evt.commands.push_back(cmd);
            script.events.push_back(evt);
        }

        {
            MissionEvent evt;
            evt.name = "orbit_insertion";
            evt.description = "Confirm orbit insertion";
            TriggerCondition trigger;
            trigger.type = TriggerType::APOAPSIS_ABOVE;
            trigger.value = 180000.0;
            evt.triggers.push_back(trigger);
            Command cmd;
            cmd.type = CommandType::LOG_MESSAGE;
            cmd.message = "Target orbit achieved - stable orbit confirmed";
            evt.commands.push_back(cmd);
            script.events.push_back(evt);
        }

        return script;
    }

    static MissionScript CreateDamageScenario() {
        MissionScript script = CreateDefaultMission();
        script.name = "Artemis II Damage Scenario";
        script.description = "Automated mission with random damage events";

        {
            MissionEvent evt;
            evt.name = "micrometeorite_impact";
            evt.description = "Micrometeorite impacts TPS";
            TriggerCondition trigger;
            trigger.type = TriggerType::TIME_ELAPSED;
            trigger.value = 30.0;
            evt.triggers.push_back(trigger);
            Command cmd;
            cmd.type = CommandType::TRIGGER_DAMAGE;
            cmd.message = "Micrometeorite impact at T+30s - TPS damaged";
            evt.commands.push_back(cmd);
            script.events.push_back(evt);
        }

        {
            MissionEvent evt;
            evt.name = "structural_stress";
            evt.description = "Max-Q structural stress event";
            TriggerCondition trigger;
            trigger.type = TriggerType::MAXQ_PASSED;
            trigger.value = 30000.0;
            evt.triggers.push_back(trigger);
            Command cmd;
            cmd.type = CommandType::TRIGGER_DAMAGE;
            cmd.message = "Severe structural stress - hull integrity reduced";
            evt.commands.push_back(cmd);
            script.events.push_back(evt);
        }

        return script;
    }
};

}
