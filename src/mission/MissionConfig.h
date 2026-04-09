#pragma once
#include <string>
#include <map>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace DeepSpace {

class MissionConfig {
public:
    struct EngineConfig {
        double thrust_N = 0.0;
        double thrustSeaLevel_N = 0.0;
        double thrustVacuum_N = 0.0;
        int engineCount = 1;
        double seaLevelIsp_s = 0.0;
        double vacuumIsp_s = 0.0;
        double fuelRatio = 0.0;
        double oxRatio = 0.0;
        double OF_ratio = 0.0;
    };

    struct TankConfig {
        std::string name;
        double fuelMass_kg = 0.0;
        double dryMass_kg = 0.0;
        std::string propellant;
    };

    struct GuidanceConfig {
        double pitchStartAlt_m = 2000.0;
        double pitchEndAlt_m = 20000.0;
        double orbitTolerance_m = 10000.0;
    };

    struct LaunchWindowConfig {
        std::string start;
        std::string end;
        bool autoCalculate = false;
    };

    struct LaunchLocationConfig {
        std::string name;
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude_m = 0.0;
        std::string timezone;
        std::string pad;
    };

    struct LaunchConfig {
        std::string datetime;
        std::string timezone;
        LaunchWindowConfig window;
    };

    struct WeatherConfig {
        bool enabled = false;
        bool realTimeData = false;
        double temperature_C = 15.0;
        double humidity_pct = 50.0;
        double pressure_hPa = 1013.25;
        double windSpeed_ms = 0.0;
        double windDirection_deg = 0.0;
        int cloudCover_pct = 0;
        bool variationEnabled = false;
        int randomSeed = 0;
    };

    struct EngineSpec {
        std::string type;
        int count = 1;
        double thrustSL_N = 0.0;
        double thrustVac_N = 0.0;
        double ispSL_s = 0.0;
        double ispVac_s = 0.0;
        double ofRatio = 0.0;
    };

    struct TankSpec {
        std::string type;
        std::string propellant;
        double fuelMass_kg = 0.0;
        double dryMass_kg = 0.0;
    };

    struct StageSpec {
        int id = 0;
        std::string name;
        std::vector<EngineSpec> engines;
        std::vector<TankSpec> tanks;
        double separatorMass_kg = 0.0;
        bool persistent = false;
    };

    struct VehicleConfig {
        std::string name;
        std::vector<StageSpec> stages;
    };

    struct PhaseSpec {
        std::string name;
        double duration_s = 0.0;
        std::string targetCondition;
        std::vector<std::pair<double, std::string>> events;
    };

    struct FlightPlanConfig {
        std::string name;
        std::vector<PhaseSpec> phases;
    };

    std::string missionName;
    double targetAp_km = 185.0;
    double targetPe_km = 180.0;
    double maxDuration_s = 7200.0;

    EngineConfig rs25;
    EngineConfig srb;
    EngineConfig rl10;
    EngineConfig aj10;

    VehicleConfig vehicle;
    FlightPlanConfig flightPlan;

    TankConfig coreLH2;
    TankConfig coreLOX;
    TankConfig icpsLH2;
    TankConfig icpsLOX;
    TankConfig orionMMH;
    TankConfig orionNTO;

    TankConfig coreRP1;
    TankConfig coreLOXOld;
    TankConfig secondStageRP1;
    TankConfig secondStageLOX;
    EngineConfig merlin;
    EngineConfig merlinVacuum;

    GuidanceConfig guidance;
    LaunchConfig launch;
    LaunchLocationConfig launchLocation;
    WeatherConfig weather;

    static MissionConfig Load(const std::string& path) {
        MissionConfig config;
        std::map<std::string, std::map<std::string, std::string>> sections;
        std::string currentSection;
        
        std::ifstream file(path);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open config: " + path);
        }
        
        std::string line;
        while (std::getline(file, line)) {
            line.erase(0, line.find_first_not_of(" \t"));
            line.erase(line.find_last_not_of(" \t") + 1);
            
            if (line.empty() || line[0] == '#') continue;
            
            if (line[0] == '[' && line.back() == ']') {
                currentSection = line.substr(1, line.length() - 2);
                continue;
            }
            
            size_t eq = line.find('=');
            if (eq != std::string::npos) {
                std::string key = line.substr(0, eq);
                std::string val = line.substr(eq + 1);
                sections[currentSection][key] = val;
            }
        }
        
        auto& m = sections["mission"];
        config.missionName = m["name"];
        config.targetAp_km = std::stod(m["targetAp_km"]);
        config.targetPe_km = std::stod(m["targetPe_km"]);
        config.maxDuration_s = std::stod(m["maxDuration_s"]);
        
        if (sections.count("rs25")) {
            auto& rs25 = sections["rs25"];
            config.rs25.thrustSeaLevel_N = std::stod(rs25["thrustSeaLevel_N"]);
            config.rs25.thrustVacuum_N = std::stod(rs25["thrustVacuum_N"]);
            config.rs25.engineCount = std::stoi(rs25["engineCount"]);
            config.rs25.seaLevelIsp_s = std::stod(rs25["seaLevelIsp_s"]);
            config.rs25.vacuumIsp_s = std::stod(rs25["vacuumIsp_s"]);
            config.rs25.fuelRatio = std::stod(rs25["fuelRatio"]);
            config.rs25.oxRatio = std::stod(rs25["oxRatio"]);
            config.rs25.OF_ratio = std::stod(rs25["OF_ratio"]);
            config.rs25.thrust_N = config.rs25.thrustSeaLevel_N;
        }
        
        if (sections.count("srb")) {
            auto& srb = sections["srb"];
            config.srb.thrustSeaLevel_N = std::stod(srb["thrustSeaLevel_N"]);
            config.srb.thrustVacuum_N = std::stod(srb["thrustVacuum_N"]);
            config.srb.engineCount = std::stoi(srb["engineCount"]);
            if (srb.count("ispSeaLevel_s")) config.srb.seaLevelIsp_s = std::stod(srb["ispSeaLevel_s"]);
            if (srb.count("ispVacuum_s")) config.srb.vacuumIsp_s = std::stod(srb["ispVacuum_s"]);
        }
        
        auto& rl10 = sections["rl10"];
        config.rl10.thrust_N = std::stod(rl10["thrust_N"]);
        config.rl10.seaLevelIsp_s = std::stod(rl10["seaLevelIsp_s"]);
        config.rl10.vacuumIsp_s = std::stod(rl10["vacuumIsp_s"]);
        config.rl10.fuelRatio = std::stod(rl10["fuelRatio"]);
        config.rl10.oxRatio = std::stod(rl10["oxRatio"]);
        config.rl10.OF_ratio = std::stod(rl10["OF_ratio"]);
        
        auto& aj10 = sections["aj10"];
        config.aj10.thrust_N = std::stod(aj10["thrust_N"]);
        config.aj10.seaLevelIsp_s = std::stod(aj10["seaLevelIsp_s"]);
        config.aj10.vacuumIsp_s = std::stod(aj10["vacuumIsp_s"]);
        config.aj10.fuelRatio = std::stod(aj10["fuelRatio"]);
        config.aj10.oxRatio = std::stod(aj10["oxRatio"]);
        config.aj10.OF_ratio = std::stod(aj10["OF_ratio"]);
        
        if (sections.count("core_stage")) {
            auto& cs = sections["core_stage"];
            config.coreLH2.name = "SLS Core LH2 Tank";
            config.coreLH2.fuelMass_kg = std::stod(cs["lh2Mass_kg"]);
            config.coreLH2.dryMass_kg = std::stod(cs["lh2Dry_kg"]);
            config.coreLH2.propellant = "LH2";
            config.coreLOX.name = "SLS Core LOX Tank";
            config.coreLOX.fuelMass_kg = std::stod(cs["loxMass_kg"]);
            config.coreLOX.dryMass_kg = std::stod(cs["loxDry_kg"]);
            config.coreLOX.propellant = "LOX";
        } else if (sections.count("core_tanks")) {
            auto& ct = sections["core_tanks"];
            config.coreRP1.name = "F9 S1 RP-1 Tank";
            config.coreRP1.fuelMass_kg = std::stod(ct["rp1Mass_kg"]);
            config.coreRP1.dryMass_kg = std::stod(ct["rp1Dry_kg"]);
            config.coreRP1.propellant = "RP1";
            config.coreLOXOld.name = "F9 S1 LOX Tank";
            config.coreLOXOld.fuelMass_kg = std::stod(ct["loxMass_kg"]);
            config.coreLOXOld.dryMass_kg = std::stod(ct["loxDry_kg"]);
            config.coreLOXOld.propellant = "LOX";
        }
        
        if (sections.count("second_stage_tanks")) {
            auto& st = sections["second_stage_tanks"];
            config.secondStageRP1.name = "F9 S2 RP-1 Tank";
            config.secondStageRP1.fuelMass_kg = std::stod(st["rp1Mass_kg"]);
            config.secondStageRP1.dryMass_kg = std::stod(st["rp1Dry_kg"]);
            config.secondStageRP1.propellant = "RP1";
            config.secondStageLOX.name = "F9 S2 LOX Tank";
            config.secondStageLOX.fuelMass_kg = std::stod(st["loxMass_kg"]);
            config.secondStageLOX.dryMass_kg = std::stod(st["loxDry_kg"]);
            config.secondStageLOX.propellant = "LOX";
        }
        
        if (sections.count("icps_tanks")) {
            auto& it = sections["icps_tanks"];
            config.icpsLH2.name = "ICPS LH2 Tank";
            config.icpsLH2.fuelMass_kg = std::stod(it["lh2Mass_kg"]);
            config.icpsLH2.dryMass_kg = std::stod(it["lh2Dry_kg"]);
            config.icpsLH2.propellant = "LH2";
            config.icpsLOX.name = "ICPS LOX Tank";
            config.icpsLOX.fuelMass_kg = std::stod(it["loxMass_kg"]);
            config.icpsLOX.dryMass_kg = std::stod(it["loxDry_kg"]);
            config.icpsLOX.propellant = "LOX";
        }
        
        if (sections.count("orion_tanks")) {
            auto& ot = sections["orion_tanks"];
            config.orionMMH.name = "Orion MMH Tank";
            config.orionMMH.fuelMass_kg = std::stod(ot["mmhMass_kg"]);
            config.orionMMH.dryMass_kg = std::stod(ot["mmhDry_kg"]);
            config.orionMMH.propellant = "MMH";
            config.orionNTO.name = "Orion NTO Tank";
            config.orionNTO.fuelMass_kg = std::stod(ot["ntoMass_kg"]);
            config.orionNTO.dryMass_kg = std::stod(ot["ntoDry_kg"]);
            config.orionNTO.propellant = "NTO";
        }
        
        if (sections.count("merlin")) {
            auto& merlin = sections["merlin"];
            config.merlin.thrustSeaLevel_N = std::stod(merlin["thrustSeaLevel_N"]);
            config.merlin.thrustVacuum_N = std::stod(merlin["thrustVacuum_N"]);
            config.merlin.engineCount = std::stoi(merlin["engineCount"]);
            config.merlin.seaLevelIsp_s = std::stod(merlin["seaLevelIsp_s"]);
            config.merlin.vacuumIsp_s = std::stod(merlin["vacuumIsp_s"]);
            config.merlin.fuelRatio = std::stod(merlin["fuelRatio"]);
            config.merlin.oxRatio = std::stod(merlin["oxRatio"]);
            config.merlin.OF_ratio = std::stod(merlin["OF_ratio"]);
            config.merlin.thrust_N = config.merlin.thrustSeaLevel_N;
        }
        
        if (sections.count("merlin_vacuum")) {
            auto& mv = sections["merlin_vacuum"];
            config.merlinVacuum.thrust_N = std::stod(mv["thrust_N"]);
            config.merlinVacuum.vacuumIsp_s = std::stod(mv["vacuumIsp_s"]);
            config.merlinVacuum.fuelRatio = std::stod(mv["fuelRatio"]);
            config.merlinVacuum.oxRatio = std::stod(mv["oxRatio"]);
            config.merlinVacuum.OF_ratio = std::stod(mv["OF_ratio"]);
        } else {
            config.merlinVacuum.thrust_N = 934000.0;
            config.merlinVacuum.vacuumIsp_s = 348.0;
            config.merlinVacuum.fuelRatio = 0.299;
            config.merlinVacuum.oxRatio = 0.701;
            config.merlinVacuum.OF_ratio = 2.35;
        }
        
        auto& g = sections["guidance"];
        config.guidance.pitchStartAlt_m = std::stod(g["pitchStartAlt_m"]);
        config.guidance.pitchEndAlt_m = std::stod(g["pitchEndAlt_m"]);
        config.guidance.orbitTolerance_m = std::stod(g["orbitTolerance_m"]);
        
        if (sections.count("launch")) {
            auto& lc = sections["launch"];
            if (lc.count("datetime")) config.launch.datetime = lc["datetime"];
            if (lc.count("timezone")) config.launch.timezone = lc["timezone"];
            if (lc.count("window_start")) config.launch.window.start = lc["window_start"];
            if (lc.count("window_end")) config.launch.window.end = lc["window_end"];
            if (lc.count("auto_calculate_window")) 
                config.launch.window.autoCalculate = lc["auto_calculate_window"] == "true";
        }
        
        if (sections.count("launch_site")) {
            auto& ls = sections["launch_site"];
            if (ls.count("name")) config.launchLocation.name = ls["name"];
            if (ls.count("latitude")) config.launchLocation.latitude = std::stod(ls["latitude"]);
            if (ls.count("longitude")) config.launchLocation.longitude = std::stod(ls["longitude"]);
            if (ls.count("altitude_m")) config.launchLocation.altitude_m = std::stod(ls["altitude_m"]);
            if (ls.count("timezone")) config.launchLocation.timezone = ls["timezone"];
            if (ls.count("pad")) config.launchLocation.pad = ls["pad"];
        }
        
        if (sections.count("weather")) {
            auto& w = sections["weather"];
            if (w.count("enabled")) config.weather.enabled = w["enabled"] == "true";
            if (w.count("real_time_data")) config.weather.realTimeData = w["real_time_data"] == "true";
            if (w.count("temperature_C")) config.weather.temperature_C = std::stod(w["temperature_C"]);
            if (w.count("humidity_pct")) config.weather.humidity_pct = std::stod(w["humidity_pct"]);
            if (w.count("pressure_hPa")) config.weather.pressure_hPa = std::stod(w["pressure_hPa"]);
            if (w.count("wind_speed_ms")) config.weather.windSpeed_ms = std::stod(w["wind_speed_ms"]);
            if (w.count("wind_direction_deg")) config.weather.windDirection_deg = std::stod(w["wind_direction_deg"]);
            if (w.count("cloud_cover_pct")) config.weather.cloudCover_pct = std::stoi(w["cloud_cover_pct"]);
            if (w.count("variation_enabled")) config.weather.variationEnabled = w["variation_enabled"] == "true";
            if (w.count("random_seed")) config.weather.randomSeed = std::stoi(w["random_seed"]);
        }
        
        return config;
    }
};

}
