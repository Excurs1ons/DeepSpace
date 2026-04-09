#pragma once
#include <memory>
#include <vector>
#include "Part.h"

namespace DeepSpace {

    struct PartData {
        std::string name;
        double mass;
        double fuelCapacity;
        double maxThrustVac;
        double ispSL;
        double ispVac;
    };

    class PartLibrary {
    public:
        static std::shared_ptr<EnginePart> CreateMerlin1D(double thrustSL_N = 845000.0,
            double seaLevelIsp_s = 282.0, double vacuumIsp_s = 311.0, double OF_ratio = 2.56) {
            return std::make_shared<EnginePart>(
                "Merlin 1D",
                470.0,
                thrustSL_N,
                seaLevelIsp_s,
                vacuumIsp_s,
                PropellantType::RP1,
                PropellantType::LOX,
                OF_ratio);
        }

        static std::shared_ptr<EnginePart> CreateMerlin1DVac(double thrustVacuum_N = 934000.0,
            double vacuumIsp_s = 348.0, double seaLevelIsp_s = 282.0, double OF_ratio = 2.35) {
            double thrustSeaLevel_N = thrustVacuum_N * (seaLevelIsp_s / vacuumIsp_s);
            return std::make_shared<EnginePart>(
                "Merlin 1D Vac",
                490.0,
                thrustSeaLevel_N,
                seaLevelIsp_s,
                vacuumIsp_s,
                PropellantType::RP1,
                PropellantType::LOX,
                OF_ratio);
        }

        static std::shared_ptr<EnginePart> CreateF1(double thrustSL_N = 7770000.0,
            double seaLevelIsp_s = 263.0, double vacuumIsp_s = 304.0, double OF_ratio = 2.27) {
            return std::make_shared<EnginePart>(
                "F-1 Engine",
                8400.0,
                thrustSL_N,
                seaLevelIsp_s,
                vacuumIsp_s,
                PropellantType::RP1,
                PropellantType::LOX,
                OF_ratio);
        }

        static std::shared_ptr<EnginePart> CreateRL10B2(double thrustSL_N = 110000.0,
            double seaLevelIsp_s = 200.0, double vacuumIsp_s = 448.0, double OF_ratio = 5.5) {
            return std::make_shared<EnginePart>(
                "RL10B-2",
                301.0,
                thrustSL_N,
                seaLevelIsp_s,
                vacuumIsp_s,
                PropellantType::LH2,
                PropellantType::LOX,
                OF_ratio);
        }

        static std::shared_ptr<EnginePart> CreateAJ10_190(double thrustSL_N = 267000.0,
            double seaLevelIsp_s = 319.0, double vacuumIsp_s = 319.0, double OF_ratio = 1.65) {
            return std::make_shared<EnginePart>(
                "AJ10-190",
                112.0,
                thrustSL_N,
                seaLevelIsp_s,
                vacuumIsp_s,
                PropellantType::MMH,
                PropellantType::NTO,
                OF_ratio);
        }

        static std::shared_ptr<FuelTankPart> CreateFalcon9S1RP1Tank(double dryMass_kg = 12000.0,
            double fuelMass_kg = 70000.0) {
            return std::make_shared<FuelTankPart>("F9 S1 RP-1 Tank", dryMass_kg, fuelMass_kg, PropellantType::RP1);
        }

        static std::shared_ptr<FuelTankPart> CreateFalcon9S1LOXTank(double dryMass_kg = 13000.0,
            double fuelMass_kg = 70000.0) {
            return std::make_shared<FuelTankPart>("F9 S1 LOX Tank", dryMass_kg, fuelMass_kg, PropellantType::LOX);
        }

        static std::shared_ptr<FuelTankPart> CreateFalcon9S2RP1Tank(double dryMass_kg = 1800.0, double fuelMass_kg = 29000.0) {
            return std::make_shared<FuelTankPart>("F9 S2 RP-1 Tank", dryMass_kg, fuelMass_kg, PropellantType::RP1);
        }

        static std::shared_ptr<FuelTankPart> CreateFalcon9S2LOXTank(double dryMass_kg = 2200.0, double fuelMass_kg = 71000.0) {
            return std::make_shared<FuelTankPart>("F9 S2 LOX Tank", dryMass_kg, fuelMass_kg, PropellantType::LOX);
        }

        static std::shared_ptr<FuelTankPart> CreateArtemis2ICPSLH2Tank(double dryMass_kg = 3200.0,
            double fuelMass_kg = 150000.0) {
            return std::make_shared<FuelTankPart>("ICPS LH2 Tank", dryMass_kg, fuelMass_kg, PropellantType::LH2);
        }

        static std::shared_ptr<FuelTankPart> CreateArtemis2ICPSLOXTank(double dryMass_kg = 4100.0,
            double fuelMass_kg = 825000.0) {
            return std::make_shared<FuelTankPart>("ICPS LOX Tank", dryMass_kg, fuelMass_kg, PropellantType::LOX);
        }

        static std::shared_ptr<FuelTankPart> CreateArtemis2OrionMMHTank(double dryMass_kg = 850.0,
            double fuelMass_kg = 3200.0) {
            return std::make_shared<FuelTankPart>("Orion MMH Tank", dryMass_kg, fuelMass_kg, PropellantType::MMH);
        }

        static std::shared_ptr<FuelTankPart> CreateArtemis2OrionNTOTank(double dryMass_kg = 900.0,
            double fuelMass_kg = 5300.0) {
            return std::make_shared<FuelTankPart>("Orion NTO Tank", dryMass_kg, fuelMass_kg, PropellantType::NTO);
        }

        // ===== SLS Block 1 Components =====
        
        // RS-25 Engine (Space Shuttle heritage, 4 on SLS Core Stage)
        static std::shared_ptr<EnginePart> CreateRS25(double thrustSL_N = 1860000.0,
            double seaLevelIsp_s = 366.0, double vacuumIsp_s = 452.0, double OF_ratio = 6.0) {
            return std::make_shared<EnginePart>(
                "RS-25",
                3515.0,  // dry mass 7,750 lbs = 3,515 kg
                thrustSL_N,
                seaLevelIsp_s,
                vacuumIsp_s,
                PropellantType::LH2,
                PropellantType::LOX,
                OF_ratio);
        }

        // Solid Rocket Booster (5-segment, 2 on SLS)
        static std::shared_ptr<EnginePart> CreateSLSRB(double thrustSL_N = 14679000.0,
            double seaLevelIsp_s = 250.0, double vacuumIsp_s = 280.0, double OF_ratio = 0.0) {
            // SRB uses solid propellant - O/F is different
            return std::make_shared<EnginePart>(
                "SLS SRB",
                75000.0,  // approximate dry mass
                thrustSL_N,
                seaLevelIsp_s,
                vacuumIsp_s,
                PropellantType::Solid,
                PropellantType::Solid,
                0.0);  // solid propellant doesn't use O/F ratio
        }

        // RL10C-2 Engine (ICPS - Artemis II)
        static std::shared_ptr<EnginePart> CreateRL10C2(double thrustSL_N = 110000.0,
            double seaLevelIsp_s = 200.0, double vacuumIsp_s = 465.0, double OF_ratio = 5.5) {
            return std::make_shared<EnginePart>(
                "RL10C-2",
                301.0,
                thrustSL_N,
                seaLevelIsp_s,
                vacuumIsp_s,
                PropellantType::LH2,
                PropellantType::LOX,
                OF_ratio);
        }

        static std::shared_ptr<FuelTankPart> CreateSLSLH2Tank(double dryMass_kg = 9500.0,
            double fuelMass_kg = 144000.0) {
            return std::make_shared<FuelTankPart>("SLS Core LH2 Tank", dryMass_kg, fuelMass_kg, PropellantType::LH2);
        }

        static std::shared_ptr<FuelTankPart> CreateSLSLOXTank(double dryMass_kg = 4500.0,
            double fuelMass_kg = 840000.0) {
            return std::make_shared<FuelTankPart>("SLS Core LOX Tank", dryMass_kg, fuelMass_kg, PropellantType::LOX);
        }

        static std::shared_ptr<FuelTankPart> CreateOrionMMHTankReal(double dryMass_kg = 400.0,
            double fuelMass_kg = 4300.0) {
            return std::make_shared<FuelTankPart>("Orion MMH Tank", dryMass_kg, fuelMass_kg, PropellantType::MMH);
        }

        static std::shared_ptr<FuelTankPart> CreateOrionNTOTankReal(double dryMass_kg = 400.0,
            double fuelMass_kg = 4300.0) {
            return std::make_shared<FuelTankPart>("Orion NTO Tank", dryMass_kg, fuelMass_kg, PropellantType::NTO);
        }
    };
}
