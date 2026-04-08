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
        static std::shared_ptr<EnginePart> CreateMerlin1D(double thrust_N = 3000000.0,
            double seaLevelIsp_s = 282.0, double vacuumIsp_s = 311.0, double OF_ratio = 2.56) {
            return std::make_shared<EnginePart>(
                "Merlin 1D",
                470.0,
                thrust_N,
                seaLevelIsp_s,
                vacuumIsp_s,
                PropellantType::RP1,
                PropellantType::LOX,
                OF_ratio);
        }

        static std::shared_ptr<EnginePart> CreateMerlin1DVac() {
            return std::make_shared<EnginePart>(
                "Merlin 1D Vac",
                490.0,
                981000.0,
                200.0,
                348.0,
                PropellantType::RP1,
                PropellantType::LOX,
                2.56);
        }

        static std::shared_ptr<EnginePart> CreateF1() {
            return std::make_shared<EnginePart>(
                "F-1 Engine",
                8400.0,
                7770000.0,
                263.0,
                304.0,
                PropellantType::RP1,
                PropellantType::LOX,
                2.27);
        }

        static std::shared_ptr<EnginePart> CreateRL10B2(double thrust_N = 15000000.0,
            double seaLevelIsp_s = 350.0, double vacuumIsp_s = 465.0, double OF_ratio = 5.5) {
            return std::make_shared<EnginePart>(
                "RL10B-2",
                301.0,
                thrust_N,
                seaLevelIsp_s,
                vacuumIsp_s,
                PropellantType::LH2,
                PropellantType::LOX,
                OF_ratio);
        }

        static std::shared_ptr<EnginePart> CreateAJ10_190(double thrust_N = 267000.0,
            double seaLevelIsp_s = 319.0, double vacuumIsp_s = 319.0, double OF_ratio = 1.65) {
            return std::make_shared<EnginePart>(
                "AJ10-190",
                112.0,
                thrust_N,
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

        static std::shared_ptr<FuelTankPart> CreateFalcon9S2RP1Tank() {
            return std::make_shared<FuelTankPart>("F9 S2 RP-1 Tank", 1800.0, 29000.0, PropellantType::RP1);
        }

        static std::shared_ptr<FuelTankPart> CreateFalcon9S2LOXTank() {
            return std::make_shared<FuelTankPart>("F9 S2 LOX Tank", 2200.0, 71000.0, PropellantType::LOX);
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
    };
}
