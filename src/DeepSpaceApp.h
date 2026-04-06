#pragma once
#include <PrismaEngine.h>
#include <memory>
#include <iomanip>
#include "environment/Planet.h"
#include "vessel/Vessel.h"

#include "physics/OrbitalElements.h"

#include "vessel/PartLibrary.h"
#include "physics/Aerodynamics.h"

using namespace DeepSpace;

class SimulationLayer : public Prisma::Layer {
public:
    SimulationLayer() : Layer("SimulationLayer"), 
        // Earth pressure 101325 Pa
        m_Earth("Earth", 5.9722e24, 6371000.0, Atmosphere(101325.0, 8500.0)),
        m_Vessel(std::make_shared<Vessel>("Falcon 9")) 
    {
    }

    void OnAttach() override {
        PRISMA_INFO("Simulation Layer Attached.");

        // --- Stage 0 (Second Stage) ---
        auto s2Tank = PartLibrary::CreateFalcon9S2Tank();
        s2Tank->SetStage(0);
        auto s2Engine = PartLibrary::CreateMerlin1DVac();
        s2Engine->SetStage(0);
        auto decoupler = std::make_shared<DecouplerPart>("Interstage", 500.0);
        decoupler->SetStage(0);

        m_Vessel->AddPart(s2Tank);
        m_Vessel->AddPart(s2Engine);
        m_Vessel->AddPart(decoupler);

        // --- Stage 1 (First Stage) ---
        auto s1Tank = PartLibrary::CreateFalcon9S1Tank();
        s1Tank->SetStage(1);
        m_Vessel->AddPart(s1Tank);
        
        for (int i = 0; i < 9; ++i) {
            auto engine = PartLibrary::CreateMerlin1D();
            engine->SetStage(1);
            m_Vessel->AddPart(engine);
        }

        Prisma::Vec3d startPos(0.0, m_Earth.GetRadius(), 0.0);
        m_Vessel->GetPhysicsBody().SetPosition(startPos);
        m_Vessel->GetPhysicsBody().SetOrientation(Prisma::Vec3d(0, 1, 0));
        m_Vessel->GetPhysicsBody().SetInertia(10000000.0); // Large inertia for F9

        PRISMA_INFO("Vessel '%s' built. Total Mass: %.1f kg", 
            m_Vessel->GetName().c_str(), m_Vessel->GetPhysicsBody().GetMass());

        // Ignite First Stage
        m_Vessel->ActivateNextStage();
        PRISMA_INFO("LIFTOFF! Stage 1 Ignited.");
    }

    void OnUpdate(Prisma::Timestep ts) override {
        double dt = ts.GetSeconds();
        PhysicsBody& body = m_Vessel->GetPhysicsBody();
        Prisma::Vec3d pos = body.GetPosition();
        double altitude = m_Earth.GetAltitude(pos);
        double ambientPressure = m_Earth.GetAtmosphere().GetPressure(altitude);

        // --- 1. Environmental Forces ---
        Prisma::Vec3d gravity = m_Earth.GetGravityAt(pos) * body.GetMass();
        body.AddForce(gravity);
        Aerodynamics::ApplyAerodynamics(body, m_Earth.GetAtmosphere(), altitude);

        // --- 2. Input Handling (Mocked) ---
        // Toggle RCS
        if (Prisma::Input::IsKeyPressed(Prisma::Key::R)) {
            m_Vessel->GetRCS().SetEnabled(!m_Vessel->GetRCS().IsEnabled());
        }

        // Manual Rotation (AD)
        if (Prisma::Input::IsKeyPressed(Prisma::Key::A)) m_Vessel->GetRCS().ApplyRotation(body, 1.0, dt);
        if (Prisma::Input::IsKeyPressed(Prisma::Key::D)) m_Vessel->GetRCS().ApplyRotation(body, -1.0, dt);

        // Manual Translation (WS for Forward/Backward)
        if (Prisma::Input::IsKeyPressed(Prisma::Key::W)) m_Vessel->GetRCS().ApplyTranslation(body, {0, 1, 0}, dt);
        if (Prisma::Input::IsKeyPressed(Prisma::Key::S)) m_Vessel->GetRCS().ApplyTranslation(body, {0, -1, 0}, dt);

        // Staging (Space)
        if (Prisma::Input::IsKeyPressed(Prisma::Key::Space)) {
             m_Vessel->ActivateNextStage();
        }

        // --- 3. Guidance Logic ---
        // Automatic Gravity Turn (if not manually controlled)
        if (altitude > 2000.0 && altitude < 80000.0 && !m_Stage1Separated) {
            double pitchFactor = (altitude - 2000.0) / 78000.0;
            double targetAngle = pitchFactor * (M_PI / 2.0);
            body.SetOrientation({std::sin(targetAngle), std::cos(targetAngle), 0.0});
            body.SetAngularVelocity(0);
        }

        // Auto-Circularization at Apoapsis
        auto orbit = OrbitalMechanics::CalculateElements(body.GetPosition(), body.GetVelocity(), m_Earth);
        if (m_Stage1Separated && altitude > 100000.0 && orbit.periapsis < 150000.0) {
            // Logic to burn prograde at Apoapsis
            double distToAp = std::abs(altitude - orbit.apoapsis);
            if (distToAp < 5000.0) { // Near Apoapsis
                // Point Prograde (Velocity direction)
                body.SetOrientation(body.GetVelocity().Normalized());
                // Burn!
                for (auto& part : m_Vessel->GetParts()) {
                    if (auto engine = std::dynamic_pointer_cast<EnginePart>(part)) {
                        if (engine->IsActive()) engine->SetThrottle(1.0);
                    }
                }
            } else {
                // Coasting
                for (auto& part : m_Vessel->GetParts()) {
                    if (auto engine = std::dynamic_pointer_cast<EnginePart>(part)) {
                        if (engine->IsActive()) engine->SetThrottle(0.0);
                    }
                }
            }
        }

        // --- 4. System Updates ---
        // Auto-Staging Stage 1
        auto s1Tank = std::static_pointer_cast<FuelTankPart>(m_Vessel->GetParts()[3]);
        if (!s1Tank->IsDecoupled() && s1Tank->GetCurrentFuel() <= 0.0 && !m_Stage1Separated) {
            PRISMA_INFO("MECO. Stage 1 Separation.");
            m_Vessel->ActivateNextStage(); 
            m_Stage1Separated = true;
            m_Vessel->GetRCS().SetEnabled(true); // Enable RCS for Stage 2
        }

        m_Vessel->Update(dt, ambientPressure);
        body.Update(dt);

        if (m_Earth.GetAltitude(body.GetPosition()) < 0) {
            body.SetPosition(body.GetPosition().Normalized() * m_Earth.GetRadius());
            body.SetVelocity({0,0,0});
        }

        // --- 5. Telemetry ---
        m_Timer += dt;
        if (m_Timer >= 2.0f) {
            double currentVel = body.GetVelocity().Length();
            double mach = Aerodynamics::GetMachNumber(currentVel, Aerodynamics::GetSpeedOfSound(altitude));
            
            // Calculate total thrust
            double totalThrust = 0;
            int activeCount = 0;
            for (auto& part : m_Vessel->GetParts()) {
                if (auto eng = std::dynamic_pointer_cast<EnginePart>(part)) {
                    if (eng->IsActive()) {
                        totalThrust += eng->GetThrust(ambientPressure);
                        activeCount++;
                    }
                }
            }
            
            PRISMA_TRACE("[Telemetry] Alt: %6.0f m | Vel: %5.0f m/s | Ap: %6.0f km | Pe: %6.0f km | Mass: %6.0f kg | Engines: %d", 
                altitude, currentVel, orbit.apoapsis / 1000.0, orbit.periapsis / 1000.0, body.GetMass(), activeCount);
            
            m_Timer = 0.0f;
        }
    }

private:
    Planet m_Earth;
    std::shared_ptr<Vessel> m_Vessel;
    float m_Timer = 0.0f;
    bool m_Stage1Separated = false;
};

class DeepSpaceApp : public Prisma::Application {
public:
    DeepSpaceApp();
    ~DeepSpaceApp() override;

    int OnInitialize() override;
    void OnUpdate(Prisma::Timestep ts) override;
    void OnRender() override;
    void OnEvent(Prisma::Event& e) override;
};
