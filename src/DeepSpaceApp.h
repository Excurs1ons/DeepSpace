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
          m_Vessel(std::make_shared<Vessel>("Artemis II Mission")) {}

    void OnAttach() override {
        PRISMA_INFO("Simulation Layer Attached");
        BuildArtemis2FlightPlan();

        auto& body = m_Vessel->GetPhysicsBody();
        body.SetPosition({0.0, m_Earth.GetRadius(), 0.0});
        body.SetOrientation({0.0, 1.0, 0.0});
        body.SetInertia(12000000.0);

        m_Vessel->ActivateNextStage();
        PRISMA_INFO("T-0: Artemis II ascent stage ignition");
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
            ApplyArtemisAscentGuidance(altitude, body);
        }

        const auto elements = OrbitalMechanics::CalculateElements(body.GetPosition(), body.GetVelocity(), m_Earth);

        if (m_BoosterCoreLoxTank && !m_BoosterCoreLoxTank->IsDecoupled() && m_BoosterCoreLoxTank->GetCurrentFuel() <= 0.0 && !m_BoosterSeparated) {
            PRISMA_INFO("Booster/core depletion - staging to ICPS");
            m_Vessel->ActivateNextStage();
            m_BoosterSeparated = true;
            m_Vessel->GetRCS().SetEnabled(true);
            m_RCSState = true;
        }

        if (m_AutopilotCircularize && m_BoosterSeparated) {
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
    void BuildArtemis2FlightPlan() {
        auto icpsLh2 = PartLibrary::CreateArtemis2ICPSLH2Tank();
        auto icpsLox = PartLibrary::CreateArtemis2ICPSLOXTank();
        auto icpsEngine = PartLibrary::CreateRL10B2();
        auto interstage = std::make_shared<DecouplerPart>("ICPS Interstage", 600.0);

        icpsLh2->SetStage(0);
        icpsLox->SetStage(0);
        icpsEngine->SetStage(0);
        interstage->SetStage(0);

        m_Vessel->AddPart(icpsLh2);
        m_Vessel->AddPart(icpsLox);
        m_Vessel->AddPart(icpsEngine);
        m_Vessel->AddPart(interstage);

        auto coreRp1 = PartLibrary::CreateFalcon9S1RP1Tank();
        auto coreLox = PartLibrary::CreateFalcon9S1LOXTank();
        coreRp1->SetStage(1);
        coreLox->SetStage(1);
        m_Vessel->AddPart(coreRp1);
        m_Vessel->AddPart(coreLox);
        m_BoosterCoreLoxTank = coreLox;

        for (int i = 0; i < 4; ++i) {
            auto engine = PartLibrary::CreateMerlin1D();
            engine->SetStage(1);
            m_Vessel->AddPart(engine);
        }

        PRISMA_INFO("Artemis II mission stack ready: %s, mass=%.0fkg", m_Vessel->GetName().c_str(), m_Vessel->GetPhysicsBody().GetMass());
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

    void ApplyArtemisAscentGuidance(double altitude, PhysicsBody& body) {
        if (altitude <= 1500.0 || altitude >= 90000.0 || m_BoosterSeparated) {
            return;
        }

        const double pitchFactor = (altitude - 1500.0) / 88500.0;
        const double targetAngle = pitchFactor * (kPi / 2.0);
        body.SetOrientation({std::sin(targetAngle), std::cos(targetAngle), 0.0});
        body.SetAngularVelocity(0.0);
    }

    void ApplyCircularizationGuidance(const OrbitalElements& orbit, double altitude, PhysicsBody& body) {
        if (!orbit.isBound) {
            return;
        }
        if (altitude <= 120000.0 || orbit.periapsis >= 185000.0) {
            return;
        }

        const double distToAp = std::abs(altitude - orbit.apoapsis);
        const bool nearApoapsis = distToAp < 4000.0;
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
            1200.0,
            1.0);

        PRISMA_TRACE(
            "[ArtemisII] Alt=%7.0fm Vel=%6.0fm/s Mach=%.2f Ap=%7.0fkm Pe=%7.0fkm PredAp=%7.0fkm PredPe=%7.0fkm Mass=%7.0fkg Eng=%d Thrust=%8.0fkN mdot=%6.1fkg/s fuel=%6.1f ox=%6.1f p=%.0fPa",
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
            status.totalMassFlow,
            status.totalFuelFlow,
            status.totalOxidizerFlow,
            ambientPressure);

        m_TelemetryTimer = 0.0;
    }

private:
    Planet m_Earth;
    std::shared_ptr<Vessel> m_Vessel;
    std::shared_ptr<FuelTankPart> m_BoosterCoreLoxTank;

    bool m_BoosterSeparated = false;
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
