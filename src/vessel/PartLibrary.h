#pragma once
#include <vector>
#include <memory>
#include "Part.h"

namespace DeepSpace {

    struct PartData {
        std::string name;
        double mass; // kg
        double fuelCapacity; // kg
        double maxThrustVac; // N
        double ispSL; // s
        double ispVac; // s
    };

    class PartLibrary {
    public:
        // SpaceX Merlin 1D (Sea Level version)
        static std::shared_ptr<EnginePart> CreateMerlin1D() {
            return std::make_shared<EnginePart>("Merlin 1D", 470.0, 914000.0, 282.0, 311.0);
        }

        // SpaceX Merlin 1D (Vacuum version)
        static std::shared_ptr<EnginePart> CreateMerlin1DVac() {
            return std::make_shared<EnginePart>("Merlin 1D Vac", 490.0, 981000.0, 200.0, 348.0);
        }

        // Rocketdyne F-1 (Saturn V Stage 1)
        static std::shared_ptr<EnginePart> CreateF1() {
            return std::make_shared<EnginePart>("F-1 Engine", 8400.0, 7770000.0, 263.0, 304.0);
        }

        // Common Fuel Tanks (Approximated)
        static std::shared_ptr<FuelTankPart> CreateFalcon9S1Tank() {
            // F9 S1 has ~400 tons of propellant
            return std::make_shared<FuelTankPart>("F9 S1 Tank", 25000.0, 400000.0);
        }

        static std::shared_ptr<FuelTankPart> CreateFalcon9S2Tank() {
            // F9 S2 has ~100 tons of propellant
            return std::make_shared<FuelTankPart>("F9 S2 Tank", 4000.0, 100000.0);
        }
    };
}
