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
        static std::shared_ptr<EnginePart> CreateMerlin1D() {
            return std::make_shared<EnginePart>(
                "Merlin 1D",
                470.0,
                914000.0,
                282.0,
                311.0,
                PropellantType::RP1,
                PropellantType::LOX,
                2.56);
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

        static std::shared_ptr<EnginePart> CreateRL10B2() {
            return std::make_shared<EnginePart>(
                "RL10B-2",
                301.0,
                110100.0,
                350.0,
                465.0,
                PropellantType::LH2,
                PropellantType::LOX,
                5.5);
        }

        static std::shared_ptr<EnginePart> CreateAJ10_190() {
            return std::make_shared<EnginePart>(
                "AJ10-190",
                112.0,
                26700.0,
                319.0,
                319.0,
                PropellantType::MMH,
                PropellantType::NTO,
                1.65);
        }

        static std::shared_ptr<FuelTankPart> CreateFalcon9S1RP1Tank() {
            return std::make_shared<FuelTankPart>("F9 S1 RP-1 Tank", 12000.0, 115000.0, PropellantType::RP1);
        }

        static std::shared_ptr<FuelTankPart> CreateFalcon9S1LOXTank() {
            return std::make_shared<FuelTankPart>("F9 S1 LOX Tank", 13000.0, 285000.0, PropellantType::LOX);
        }

        static std::shared_ptr<FuelTankPart> CreateFalcon9S2RP1Tank() {
            return std::make_shared<FuelTankPart>("F9 S2 RP-1 Tank", 1800.0, 29000.0, PropellantType::RP1);
        }

        static std::shared_ptr<FuelTankPart> CreateFalcon9S2LOXTank() {
            return std::make_shared<FuelTankPart>("F9 S2 LOX Tank", 2200.0, 71000.0, PropellantType::LOX);
        }

        static std::shared_ptr<FuelTankPart> CreateArtemis2ICPSLH2Tank() {
            return std::make_shared<FuelTankPart>("ICPS LH2 Tank", 3200.0, 22000.0, PropellantType::LH2);
        }

        static std::shared_ptr<FuelTankPart> CreateArtemis2ICPSLOXTank() {
            return std::make_shared<FuelTankPart>("ICPS LOX Tank", 4100.0, 120000.0, PropellantType::LOX);
        }

        static std::shared_ptr<FuelTankPart> CreateArtemis2OrionMMHTank() {
            return std::make_shared<FuelTankPart>("Orion MMH Tank", 850.0, 3200.0, PropellantType::MMH);
        }

        static std::shared_ptr<FuelTankPart> CreateArtemis2OrionNTOTank() {
            return std::make_shared<FuelTankPart>("Orion NTO Tank", 900.0, 5300.0, PropellantType::NTO);
        }
    };
}
