#pragma once
#include <PrismaEngine.h>
#include <memory>
#include <iomanip>
#include "environment/Planet.h"
#include "vessel/Vessel.h"
#include "physics/OrbitalElements.h"
#include "vessel/PartLibrary.h"
#include "physics/Aerodynamics.h"

namespace Prisma {
    namespace Input {
        inline bool IsKeyPressed(int) { return false; }
    }
    enum Key { A, D };
}

using namespace DeepSpace;

class SimulationLayer : public Prisma::Layer {
public:
    SimulationLayer() : Layer("SimulationLayer"), 
        m_Earth("Earth", 5.9722e24, 6371000.0, Atmosphere(101325.0, 8500.0)),
        m_Vessel(std::make_shared<Vessel>("Falcon 9")) {}

    void OnAttach() override {
        auto s2Tank = PartLibrary::CreateFalcon9S2Tank(); s2Tank->SetStage(0);
        auto s2Engine = PartLibrary::CreateMerlin1DVac(); s2Engine->SetStage(0);
        auto decoupler = std::make_shared<DecouplerPart>("Interstage", 500.0); decoupler->SetStage(0);
        m_Vessel->AddPart(s2Tank); m_Vessel->AddPart(s2Engine); m_Vessel->AddPart(decoupler);

        auto s1Tank = PartLibrary::CreateFalcon9S1Tank(); s1Tank->SetStage(1); m_Vessel->AddPart(s1Tank);
        for (int i = 0; i < 9; ++i) { auto e = PartLibrary::CreateMerlin1D(); e->SetStage(1); m_Vessel->AddPart(e); }

        m_Vessel->GetPhysicsBody().SetPosition({0.0, m_Earth.GetRadius(), 0.0});
        m_Vessel->GetPhysicsBody().SetOrientation({0, 1, 0});
        m_Vessel->GetPhysicsBody().SetInertia(10000000.0);
        m_Vessel->ActivateNextStage();
    }

    void OnUpdate(Prisma::Timestep ts) override {
        double dt = ts.GetSeconds();
        PhysicsBody& body = m_Vessel->GetPhysicsBody();
        double altitude = m_Earth.GetAltitude(body.GetPosition());
        double ambientPressure = m_Earth.GetAtmosphere().GetPressure(altitude);

        body.AddForce(m_Earth.GetGravityAt(body.GetPosition()) * body.GetMass());
        Aerodynamics::ApplyAerodynamics(body, m_Earth.GetAtmosphere(), altitude);

        if (Prisma::Input::IsKeyPressed(Prisma::Key::A)) m_Vessel->GetRCS().ApplyRotation(body, 1.0, dt);
        if (Prisma::Input::IsKeyPressed(Prisma::Key::D)) m_Vessel->GetRCS().ApplyRotation(body, -1.0, dt);

        if (altitude > 2000.0 && altitude < 80000.0 && !m_Stage1Separated) {
            double pitch = (altitude - 2000.0) / 78000.0 * (M_PI / 2.0);
            body.SetOrientation({std::sin(pitch), std::cos(pitch), 0.0});
            body.SetAngularVelocity(0);
        }

        auto orbit = OrbitalMechanics::CalculateElements(body.GetPosition(), body.GetVelocity(), m_Earth);
        if (m_Stage1Separated && altitude > 100000.0 && orbit.periapsis < 150000.0) {
            if (std::abs(altitude - orbit.apoapsis) < 5000.0) {
                body.SetOrientation(body.GetVelocity().Normalized());
                for (auto& p : m_Vessel->GetParts()) if (auto e = std::dynamic_pointer_cast<EnginePart>(p)) if (e->IsActive()) e->SetThrottle(1.0);
            } else {
                for (auto& p : m_Vessel->GetParts()) if (auto e = std::dynamic_pointer_cast<EnginePart>(p)) if (e->IsActive()) e->SetThrottle(0.0);
            }
        }

        auto s1Tank = std::static_pointer_cast<FuelTankPart>(m_Vessel->GetParts()[3]);
        if (!s1Tank->IsDecoupled() && s1Tank->GetCurrentFuel() <= 0.0 && !m_Stage1Separated) {
            m_Vessel->ActivateNextStage(); m_Stage1Separated = true; m_Vessel->GetRCS().SetEnabled(true);
        }

        m_Vessel->Update(dt, ambientPressure);
        body.Update(dt);

        m_Timer += dt;
        if (m_Timer >= 5.0f) {
            PRISMA_TRACE("Alt: %.0fm | Vel: %.0fm/s | Ap: %.0fkm | Pe: %.0fkm", 
                altitude, body.GetVelocity().Length(), orbit.apoapsis/1000, orbit.periapsis/1000);
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
    DeepSpaceApp() {}
    int OnInitialize() override { PushLayer(new SimulationLayer()); return 0; }
};
