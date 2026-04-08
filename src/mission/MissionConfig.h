#pragma once
#include <string>
#include <map>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace DeepSpace {

class MissionConfig {
public:
    struct EngineConfig {
        double thrust_N = 0.0;
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

    std::string missionName;
    double targetAp_km = 185.0;
    double targetPe_km = 180.0;
    double maxDuration_s = 7200.0;

    EngineConfig merlin;
    EngineConfig rl10;
    EngineConfig aj10;

    TankConfig coreRP1;
    TankConfig coreLOX;
    TankConfig icpsLH2;
    TankConfig icpsLOX;
    TankConfig orionMMH;
    TankConfig orionNTO;

    GuidanceConfig guidance;

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
        
        auto& merlin = sections["merlin"];
        config.merlin.thrust_N = std::stod(merlin["thrust_N"]);
        config.merlin.seaLevelIsp_s = std::stod(merlin["seaLevelIsp_s"]);
        config.merlin.vacuumIsp_s = std::stod(merlin["vacuumIsp_s"]);
        config.merlin.fuelRatio = std::stod(merlin["fuelRatio"]);
        config.merlin.oxRatio = std::stod(merlin["oxRatio"]);
        config.merlin.OF_ratio = std::stod(merlin["OF_ratio"]);
        
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
        
        auto& ct = sections["core_tanks"];
        config.coreRP1.name = "F9 S1 RP-1 Tank";
        config.coreRP1.fuelMass_kg = std::stod(ct["rp1Mass_kg"]);
        config.coreRP1.dryMass_kg = std::stod(ct["rp1Dry_kg"]);
        config.coreRP1.propellant = "RP1";
        config.coreLOX.name = "F9 S1 LOX Tank";
        config.coreLOX.fuelMass_kg = std::stod(ct["loxMass_kg"]);
        config.coreLOX.dryMass_kg = std::stod(ct["loxDry_kg"]);
        config.coreLOX.propellant = "LOX";
        
        auto& it = sections["icps_tanks"];
        config.icpsLH2.name = "ICPS LH2 Tank";
        config.icpsLH2.fuelMass_kg = std::stod(it["lh2Mass_kg"]);
        config.icpsLH2.dryMass_kg = std::stod(it["lh2Dry_kg"]);
        config.icpsLH2.propellant = "LH2";
        config.icpsLOX.name = "ICPS LOX Tank";
        config.icpsLOX.fuelMass_kg = std::stod(it["loxMass_kg"]);
        config.icpsLOX.dryMass_kg = std::stod(it["loxDry_kg"]);
        config.icpsLOX.propellant = "LOX";
        
        auto& ot = sections["orion_tanks"];
        config.orionMMH.name = "Orion MMH Tank";
        config.orionMMH.fuelMass_kg = std::stod(ot["mmhMass_kg"]);
        config.orionMMH.dryMass_kg = std::stod(ot["mmhDry_kg"]);
        config.orionMMH.propellant = "MMH";
        config.orionNTO.name = "Orion NTO Tank";
        config.orionNTO.fuelMass_kg = std::stod(ot["ntoMass_kg"]);
        config.orionNTO.dryMass_kg = std::stod(ot["ntoDry_kg"]);
        config.orionNTO.propellant = "NTO";
        
        auto& g = sections["guidance"];
        config.guidance.pitchStartAlt_m = std::stod(g["pitchStartAlt_m"]);
        config.guidance.pitchEndAlt_m = std::stod(g["pitchEndAlt_m"]);
        config.guidance.orbitTolerance_m = std::stod(g["orbitTolerance_m"]);
        
        return config;
    }
};

}
