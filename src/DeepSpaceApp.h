#pragma once
#include <PrismaEngine.h>
#include <cmath>
#include <memory>
#include "environment/Planet.h"
#include "physics/Aerodynamics.h"
#include "physics/OrbitalElements.h"
#include "vessel/PartLibrary.h"
#include "vessel/Vessel.h"

using namespace DeepSpace;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

class SimulationLayer : public Prisma::Layer {
public:
    SimulationLayer()
        : Layer("SimulationLayer"),
          m_Earth("Earth", 5.9722e24, 6371000.0, Atmosphere(101325.0, 8500.0)),
          m_Vessel(std::make_shared<Vessel>("Falcon 9")) {}

    void OnAttach() override {
        PRISMA_INFO("Simulation Layer Attached");
        BuildFalcon9();

        auto& body = m_Vessel->GetPhysicsBody();
        body.SetPosition({0.0, m_Earth.GetRadius(), 0.0});
        body.SetOrientation({0.0, 1.0, 0.0});
        body.SetInertia(10000000.0);

        m_Vessel->ActivateNextStage();
        PRISMA_INFO("LIFTOFF - stage 1 ignition");
    }

    void OnUpdate(Prisma::Timestep ts) override {
        const double dt = ts.GetSeconds();
        if (dt <= 0.0) return;

        PhysicsBody& body = m_Vessel->GetPhysicsBody();
        const Prisma::Vec3d pos = body.GetPosition();
        const double altitude = m_Earth.GetAltitude(pos);
        const double ambientPressure = m_Earth.GetAtmosphere().GetPressure(altitude);

        body.AddForce(m_Earth.GetGravityAt(pos) * body.GetMass());
        Aerodynamics::ApplyAerodynamics(body, m_Earth.GetAtmosphere(), altitude);

        HandleInput(dt);

        if (!m_ManualControlEnabled) {
            ApplyAscentGuidance(altitude, body);
        }

        const auto elements = OrbitalMechanics::CalculateElements(body.GetPosition(), body.GetVelocity(), m_Earth);

        if (m_Stage1Tank && !m_Stage1Tank->IsDecoupled() && m_Stage1Tank->GetCurrentFuel() <= 0.0 && !m_Stage1Separated) {
            PRISMA_INFO("MECO - stage separation");
            m_Vessel->ActivateNextStage();
            m_Stage1Separated = true;
            m_Vessel->GetRCS().SetEnabled(true);
            m_RCSState = true;
        }

        if (m_AutopilotCircularize && m_Stage1Separated) {
            ApplyCircularizationGuidance(elements, altitude, body);
        }

        const EngineStatus status = m_Vessel->Update(dt, ambientPressure);
        body.Update(dt);

        if (m_Earth.GetAltitude(body.GetPosition()) < 0.0) {
            body.SetPosition(body.GetPosition().Normalized() * m_Earth.GetRadius());
            body.SetVelocity({0.0, 0.0, 0.0});
        }

        EmitTelemetry(altitude, ambientPressure, elements, status, dt);
    }

private:
    void BuildFalcon9() {
        auto s2Tank = PartLibrary::CreateFalcon9S2Tank();
        auto s2Engine = PartLibrary::CreateMerlin1DVac();
        auto decoupler = std::make_shared<DecouplerPart>("Interstage", 500.0);

        s2Tank->SetStage(0);
        s2Engine->SetStage(0);
        decoupler->SetStage(0);

        m_Vessel->AddPart(s2Tank);
        m_Vessel->AddPart(s2Engine);
        m_Vessel->AddPart(decoupler);

        auto s1Tank = PartLibrary::CreateFalcon9S1Tank();
        s1Tank->SetStage(1);
        m_Vessel->AddPart(s1Tank);
        m_Stage1Tank = s1Tank;

        for (int i = 0; i < 9; ++i) {
            auto engine = PartLibrary::CreateMerlin1D();
            engine->SetStage(1);
            m_Vessel->AddPart(engine);
        }

        PRISMA_INFO("Vessel built: %s, mass=%.0fkg", m_Vessel->GetName().c_str(), m_Vessel->GetPhysicsBody().GetMass());
    }

    void HandleInput(double dt) {
        auto input = Prisma::Engine::Get().GetInputManager();

        const bool rPressed = input->IsKeyPressed(Prisma::Input::KeyCode::R);
        const bool spacePressed = input->IsKeyPressed(Prisma::Input::KeyCode::Space);
        const bool cPressed = input->IsKeyPressed(Prisma::Input::KeyCode::C);
        const bool mPressed = input->IsKeyPressed(Prisma::Input::KeyCode::M);

        if (rPressed && !m_RPressedLastFrame) {
            m_RCSState = !m_RCSState;
            m_Vessel->GetRCS().SetEnabled(m_RCSState);
            PRISMA_INFO("RCS %s", m_RCSState ? "ON" : "OFF");
        }
        if (spacePressed && !m_SpacePressedLastFrame) {
            m_Vessel->ActivateNextStage();
            PRISMA_INFO("Manual staging triggered");
        }
        if (cPressed && !m_CPressedLastFrame) {
            m_AutopilotCircularize = !m_AutopilotCircularize;
            PRISMA_INFO("Circularization autopilot %s", m_AutopilotCircularize ? "ON" : "OFF");
        }
        if (mPressed && !m_MPressedLastFrame) {
            m_ManualControlEnabled = !m_ManualControlEnabled;
            PRISMA_INFO("Manual attitude mode %s", m_ManualControlEnabled ? "ON" : "OFF");
        }

        m_RPressedLastFrame = rPressed;
        m_SpacePressedLastFrame = spacePressed;
        m_CPressedLastFrame = cPressed;
        m_MPressedLastFrame = mPressed;

        if (input->IsKeyPressed(Prisma::Input::KeyCode::A)) {
            m_Vessel->GetRCS().ApplyRotation(m_Vessel->GetPhysicsBody(), 1.0, dt);
        }
        if (input->IsKeyPressed(Prisma::Input::KeyCode::D)) {
            m_Vessel->GetRCS().ApplyRotation(m_Vessel->GetPhysicsBody(), -1.0, dt);
        }
        if (input->IsKeyPressed(Prisma::Input::KeyCode::W)) {
            m_Vessel->GetRCS().ApplyTranslation(m_Vessel->GetPhysicsBody(), {0.0, 1.0, 0.0}, dt);
        }
        if (input->IsKeyPressed(Prisma::Input::KeyCode::S)) {
            m_Vessel->GetRCS().ApplyTranslation(m_Vessel->GetPhysicsBody(), {0.0, -1.0, 0.0}, dt);
        }

        m_Vessel->GetRCS().Stabilize(m_Vessel->GetPhysicsBody(), dt);
    }

    void ApplyAscentGuidance(double altitude, PhysicsBody& body) {
        if (altitude <= 2000.0 || altitude >= 80000.0 || m_Stage1Separated) {
            return;
        }

        const double pitchFactor = (altitude - 2000.0) / 78000.0;
        const double targetAngle = pitchFactor * (kPi / 2.0);
        body.SetOrientation({std::sin(targetAngle), std::cos(targetAngle), 0.0});
        body.SetAngularVelocity(0.0);
    }

    void ApplyCircularizationGuidance(const OrbitalElements& orbit, double altitude, PhysicsBody& body) {
        if (!orbit.isBound) {
            return;
        }
        if (altitude <= 100000.0 || orbit.periapsis >= 150000.0) {
            return;
        }

        const double distToAp = std::abs(altitude - orbit.apoapsis);
        const bool nearApoapsis = distToAp < 5000.0;
        const Prisma::Vec3d velocity = body.GetVelocity();
        if (velocity.Length() > 1e-3) {
            body.SetOrientation(velocity.Normalized());
        }

        for (const auto& part : m_Vessel->GetParts()) {
            auto engine = std::dynamic_pointer_cast<EnginePart>(part);
            if (!engine || !engine->IsActive()) {
                continue;
            }
            engine->SetThrottle(nearApoapsis ? 1.0 : 0.0);
        }
    }

    void EmitTelemetry(
        double altitude,
        double ambientPressure,
        const OrbitalElements& orbit,
        const EngineStatus& status,
        double dt)
    {
        m_TelemetryTimer += dt;
        if (m_TelemetryTimer < 2.0) {
            return;
        }

        const PhysicsBody& body = m_Vessel->GetPhysicsBody();
        const double speed = body.GetVelocity().Length();
        const double mach = Aerodynamics::GetMachNumber(speed, Aerodynamics::GetSpeedOfSound(altitude));

        const OrbitPrediction prediction = OrbitalMechanics::PredictVacuumExtrema(
            body.GetPosition(),
            body.GetVelocity(),
            m_Earth,
            900.0,
            1.0);

        PRISMA_TRACE(
            "[Telemetry] Alt=%7.0fm Vel=%6.0fm/s Mach=%.2f Ap=%7.0fkm Pe=%7.0fkm PredAp=%7.0fkm PredPe=%7.0fkm Mass=%7.0fkg Eng=%d Thrust=%8.0fkN p=%.0fPa",
            altitude,
            speed,
            mach,
            orbit.apoapsis / 1000.0,
            orbit.periapsis / 1000.0,
            prediction.apoapsis / 1000.0,
            prediction.periapsis / 1000.0,
            body.GetMass(),
            status.activeEngines,
            status.totalThrust / 1000.0,
            ambientPressure);

        m_TelemetryTimer = 0.0;
    }

private:
    Planet m_Earth;
    std::shared_ptr<Vessel> m_Vessel;
    std::shared_ptr<FuelTankPart> m_Stage1Tank;

    bool m_Stage1Separated = false;
    bool m_AutopilotCircularize = true;
    bool m_ManualControlEnabled = false;
    bool m_RCSState = false;

    bool m_RPressedLastFrame = false;
    bool m_SpacePressedLastFrame = false;
    bool m_CPressedLastFrame = false;
    bool m_MPressedLastFrame = false;

    double m_TelemetryTimer = 0.0;
};

class DeepSpaceApp : public Prisma::Application {
public:
    DeepSpaceApp() = default;

    int OnInitialize() override {
        PushLayer(new SimulationLayer());
        return 0;
    }
};
