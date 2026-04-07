#pragma once
#include "engine/MockEngine.h"
#include "environment/Planet.h"
#include "physics/Aerodynamics.h"
#include "physics/OrbitalElements.h"
#include "vessel/EnduranceStation.h"
#include "vessel/PartLibrary.h"
#include "vessel/Vessel.h"

namespace DeepSpace {
using Vec3d = Mock::Vec3d;

class SimulationLayer : public Mock::Layer {
public:
    SimulationLayer()
        : Layer("SimulationLayer"),
          m_Earth("Earth", 5.9722e24, 6371000.0, Atmosphere(101325.0, 8500.0)),
          m_Vessel(std::make_shared<Vessel>("Artemis II Mission")),
          m_EnduranceStation(nullptr),
          m_EnduranceMode(false),
          m_FireTriggered(false),
          m_DepressTriggered(false),
          m_ExplosionTriggered(false) {}

    void OnAttach() override {
        MOCK_INFO("Simulation Layer Attached");
        
        BuildArtemis2FlightPlan();

        auto& body = m_Vessel->GetPhysicsBody();
        body.SetPosition({0.0, m_Earth.GetRadius(), 0.0});
        body.SetOrientation({0.0, 1.0, 0.0});
        body.SetInertia(12000000.0);

        m_Vessel->ActivateNextStage();
        MOCK_INFO("T-0: Artemis II ascent stage ignition");
    }

    void OnUpdate(double dt) override {
        if (dt <= 0.0) return;

        PhysicsBody& body = m_Vessel->GetPhysicsBody();
        const Vec3d pos = body.GetPosition();
        const double altitude = m_Earth.GetAltitude(pos);
        const double ambientPressure = m_Earth.GetAtmosphere().GetPressure(altitude);

        body.AddForce(m_Earth.GetGravityAt(pos) * body.GetMass());
        Aerodynamics::ApplyAerodynamics(body, m_Earth.GetAtmosphere(), altitude);

        m_MissionTime += dt;
        HandleInput(dt);

        if (!m_ManualControlEnabled) {
            ApplyArtemisAscentGuidance(altitude, body);
        }

        const auto elements = OrbitalMechanics::CalculateElements(body.GetPosition(), body.GetVelocity(), m_Earth);
        const double dynamicPressure = 0.5 * m_Earth.GetAtmosphere().GetDensity(altitude) * body.GetVelocity().LengthSquared();

        ManageMissionEvents(altitude, dynamicPressure);

        if (m_AutopilotCircularize && m_BoosterSeparated) {
            ApplyCircularizationGuidance(elements, altitude, body);
        }

        const EngineStatus status = m_Vessel->Update(dt, ambientPressure);
        body.Update(dt);

        if (m_Earth.GetAltitude(body.GetPosition()) < 0.0) {
            body.SetPosition(body.GetPosition().Normalized() * m_Earth.GetRadius());
            body.SetVelocity({0.0, 0.0, 0.0});
        }

        if (m_EnduranceStation) {
            m_EnduranceStation->Update(dt);

            double rpm = m_EnduranceStation->GetSpinRateRpm();
            double gForce = m_EnduranceStation->GetArtificialGravityAtModule(StationModuleId::Bridge);
            MOCK_TRACE("[Endurance] RPM=%.2f g=%.2f", rpm, gForce / 9.80665);
        }

        EmitTelemetry(altitude, ambientPressure, elements, status, dynamicPressure, dt);
    }

private:
    void BuildArtemis2FlightPlan() {
        auto orionMmh = PartLibrary::CreateArtemis2OrionMMHTank();
        auto orionNto = PartLibrary::CreateArtemis2OrionNTOTank();
        auto orionAj10 = PartLibrary::CreateAJ10_190();
        orionMmh->SetStage(0);
        orionNto->SetStage(0);
        orionAj10->SetStage(0);

        auto icpsLh2 = PartLibrary::CreateArtemis2ICPSLH2Tank();
        auto icpsLox = PartLibrary::CreateArtemis2ICPSLOXTank();
        auto icpsEngine = PartLibrary::CreateRL10B2();
        auto interstage = std::make_shared<DecouplerPart>("ICPS Interstage", 600.0);

        icpsLh2->SetStage(1);
        icpsLox->SetStage(1);
        icpsEngine->SetStage(1);
        interstage->SetStage(1);

        m_Vessel->AddPart(orionMmh);
        m_Vessel->AddPart(orionNto);
        m_Vessel->AddPart(orionAj10);

        m_Vessel->AddPart(icpsLh2);
        m_Vessel->AddPart(icpsLox);
        m_Vessel->AddPart(icpsEngine);
        m_Vessel->AddPart(interstage);

        auto coreRp1 = PartLibrary::CreateFalcon9S1RP1Tank();
        auto coreLox = PartLibrary::CreateFalcon9S1LOXTank();
        coreRp1->SetStage(2);
        coreLox->SetStage(2);
        m_Vessel->AddPart(coreRp1);
        m_Vessel->AddPart(coreLox);
        m_BoosterCoreLoxTank = coreLox;

        for (int i = 0; i < 4; ++i) {
            auto engine = PartLibrary::CreateMerlin1D();
            engine->SetStage(2);
            m_Vessel->AddPart(engine);
        }

        MOCK_INFO("Artemis II stack ready: %s, mass=%.0fkg", m_Vessel->GetName().c_str(), m_Vessel->GetPhysicsBody().GetMass());
    }

    void HandleInput(double dt) {
        auto input = Mock::InputManager::Get();

        const bool rPressed = input.IsKeyPressed(Mock::KeyCode::R);
        const bool spacePressed = input.IsKeyPressed(Mock::KeyCode::Space);
        const bool cPressed = input.IsKeyPressed(Mock::KeyCode::C);
        const bool mPressed = input.IsKeyPressed(Mock::KeyCode::M);

        if (rPressed && !m_RPressedLastFrame) {
            m_RCSState = !m_RCSState;
            m_Vessel->GetRCS().SetEnabled(m_RCSState);
            MOCK_INFO("RCS %s", m_RCSState ? "ON" : "OFF");
        }
        if (spacePressed && !m_SpacePressedLastFrame) {
            m_Vessel->ActivateNextStage();
            MOCK_INFO("Manual staging triggered");
        }
        if (cPressed && !m_CPressedLastFrame) {
            m_AutopilotCircularize = !m_AutopilotCircularize;
            MOCK_INFO("Circularization autopilot %s", m_AutopilotCircularize ? "ON" : "OFF");
        }
        if (mPressed && !m_MPressedLastFrame) {
            m_EnduranceMode = !m_EnduranceMode;
            if (m_EnduranceMode) {
                m_EnduranceStation = std::make_shared<EnduranceStation>();
                m_EnduranceStation->SetPosition({0.0, 7000000.0, 0.0});
                MOCK_INFO("Endurance station deployed at orbital position");
            }
            MOCK_INFO("Endurance mode %s", m_EnduranceMode ? "ENABLED" : "DISABLED");
        }

        if (input.IsKeyPressed(Mock::KeyCode::F) && !m_FireTriggered) {
            m_FireTriggered = true;
            if (m_EnduranceStation) {
                m_EnduranceStation->TriggerFire(StationModuleId::Lab);
                MOCK_INFO("EMERGENCY: Cabin fire in Lab module!");
            }
        }
        if (input.IsKeyPressed(Mock::KeyCode::E) && !m_DepressTriggered) {
            m_DepressTriggered = true;
            if (m_EnduranceStation) {
                m_EnduranceStation->TriggerDepressurization(StationModuleId::Airlock1, 0.01);
                MOCK_INFO("EMERGENCY: Depressurization in Airlock!");
            }
        }
        if (input.IsKeyPressed(Mock::KeyCode::X) && !m_ExplosionTriggered) {
            m_ExplosionTriggered = true;
            if (m_EnduranceStation) {
                m_EnduranceStation->TriggerExplosion(StationModuleId::Airlock3);
                m_EnduranceStation->EmergencySpinUp();
                MOCK_INFO("EMERGENCY: Airlock explosion! Station spinning up!");
            }
        }

        m_RPressedLastFrame = rPressed;
        m_SpacePressedLastFrame = spacePressed;
        m_CPressedLastFrame = cPressed;
        m_MPressedLastFrame = mPressed;

        if (input.IsKeyPressed(Mock::KeyCode::A)) {
            m_Vessel->GetRCS().ApplyRotation(m_Vessel->GetPhysicsBody(), 1.0, dt);
        }
        if (input.IsKeyPressed(Mock::KeyCode::D)) {
            m_Vessel->GetRCS().ApplyRotation(m_Vessel->GetPhysicsBody(), -1.0, dt);
        }
        if (input.IsKeyPressed(Mock::KeyCode::W)) {
            m_Vessel->GetRCS().ApplyTranslation(m_Vessel->GetPhysicsBody(), {0.0, 1.0, 0.0}, dt);
        }
        if (input.IsKeyPressed(Mock::KeyCode::S)) {
            m_Vessel->GetRCS().ApplyTranslation(m_Vessel->GetPhysicsBody(), {0.0, -1.0, 0.0}, dt);
        }

        m_Vessel->GetRCS().Stabilize(m_Vessel->GetPhysicsBody(), dt);
    }

    void ManageMissionEvents(double altitude, double dynamicPressure) {
        if (!m_MaxQAnnounced) {
            if (dynamicPressure > m_MaxQObserved) {
                m_MaxQObserved = dynamicPressure;
                m_MaxQTime = m_MissionTime;
            }
            if (altitude > 18000.0 && m_MaxQObserved > 1000.0) {
                m_MaxQAnnounced = true;
                MOCK_INFO("Max-Q passed at T+%.1fs (q=%.0f Pa)", m_MaxQTime, m_MaxQObserved);
            }
        }

        if (!m_BoosterSeparated && m_BoosterCoreLoxTank && m_BoosterCoreLoxTank->GetCurrentFuel() <= 0.0) {
            MOCK_INFO("Booster/core depletion - staging to ICPS");
            m_Vessel->ActivateNextStage();
            m_BoosterSeparated = true;
            m_Vessel->GetRCS().SetEnabled(true);
            m_RCSState = true;
            m_ICPSIgnitionTime = m_MissionTime;
        }

        if (m_BoosterSeparated && !m_ICPSSettled) {
            const double coastTime = m_MissionTime - m_ICPSIgnitionTime;
            if (coastTime > 5.0) {
                m_ICPSSettled = true;
                MOCK_INFO("ICPS guidance settled at T+%.1fs", m_MissionTime);
            }
        }

        if (m_BoosterSeparated && !m_TEIWindowOpened && altitude > 185000.0) {
            m_TEIWindowOpened = true;
            MOCK_INFO("Artemis II translunar preparation window opened");
        }

        if (m_TEIWindowOpened && !m_OrionTakeover && m_MissionTime > 1200.0) {
            m_OrionTakeover = true;
            m_Vessel->ActivateNextStage();
            MOCK_INFO("Orion service module propulsion takeover (AJ10)");
        }
    }

    void ApplyArtemisAscentGuidance(double altitude, PhysicsBody& body) {
        if (altitude <= 1500.0 || altitude >= 90000.0 || m_BoosterSeparated) {
            return;
        }

        const double pitchFactor = (altitude - 1500.0) / 88500.0;
        const double targetAngle = pitchFactor * (3.14159265358979323846 / 2.0);
        body.SetOrientation({std::sin(targetAngle), std::cos(targetAngle), 0.0});
        body.SetAngularVelocity(0.0);

        if (altitude > 9500.0 && altitude < 15000.0) {
            m_Vessel->SetStageThrottle(2, 0.72);
        } else {
            m_Vessel->SetStageThrottle(2, 1.0);
        }
    }

    void ApplyCircularizationGuidance(const OrbitalElements& orbit, double altitude, PhysicsBody& body) {
        if (!orbit.isBound) {
            return;
        }
        if (altitude <= 120000.0 || orbit.apoapsis >= 185000.0) {
            return;
        }

        const double distToAp = std::abs(altitude - orbit.apoapsis);
        const bool nearApoapsis = distToAp < 4000.0;
        const Vec3d velocity = body.GetVelocity();
        if (velocity.Length() > 1e-3) {
            body.SetOrientation(velocity.Normalized());
        }

        if (m_OrionTakeover) {
            m_Vessel->SetStageThrottle(0, nearApoapsis ? 0.65 : 0.0);
        } else {
            m_Vessel->SetStageThrottle(1, nearApoapsis ? 1.0 : 0.15);
        }
    }

    void EmitTelemetry(
        double altitude,
        double ambientPressure,
        const OrbitalElements& orbit,
        const EngineStatus& status,
        double dynamicPressure,
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

        const double s2Lh2 = m_Vessel->GetPropellantRemainingMass(1, PropellantType::LH2);
        const double s2Lox = m_Vessel->GetPropellantRemainingMass(1, PropellantType::LOX);
        const double smMmh = m_Vessel->GetPropellantRemainingMass(0, PropellantType::MMH);
        const double smNto = m_Vessel->GetPropellantRemainingMass(0, PropellantType::NTO);

        MOCK_TRACE(
            "[ArtemisII] T=%6.1fs Alt=%7.0fm Vel=%6.0fm/s q=%7.0fPa Mach=%.2f Ap=%7.0fkm Pe=%7.0fkm PredAp=%7.0fkm PredPe=%7.0fkm Mass=%7.0fkg Eng=%d Thr=%8.0fkN ThrPct=%.0f%% mdot=%6.1f fuel=%6.1f ox=%6.1f S1LH2=%6.0f S1LOX=%6.0f S0MMH=%6.0f S0NTO=%6.0f p=%.0fPa",
            m_MissionTime,
            altitude,
            speed,
            dynamicPressure,
            mach,
            orbit.apoapsis / 1000.0,
            orbit.periapsis / 1000.0,
            prediction.apoapsis / 1000.0,
            prediction.periapsis / 1000.0,
            body.GetMass(),
            status.activeEngines,
            status.totalThrust / 1000.0,
            status.maxThrottle * 100.0,
            status.totalMassFlow,
            status.totalFuelFlow,
            status.totalOxidizerFlow,
            s2Lh2,
            s2Lox,
            smMmh,
            smNto,
            ambientPressure);

        if (m_EnduranceStation) {
            const double rpm = m_EnduranceStation->GetSpinRateRpm();
            const double gForce = m_EnduranceStation->GetArtificialGravityAtModule(StationModuleId::Bridge);
            MOCK_TRACE("Station RPM=%.2f g=%.2f", rpm, gForce / 9.80665);
        }

        m_TelemetryTimer = 0.0;
    }

private:
    Planet m_Earth;
    std::shared_ptr<Vessel> m_Vessel;
    std::shared_ptr<FuelTankPart> m_BoosterCoreLoxTank;
    std::shared_ptr<EnduranceStation> m_EnduranceStation;
    std::vector<std::shared_ptr<Vessel>> m_Spacecraft;

    bool m_BoosterSeparated = false;
    bool m_TEIWindowOpened = false;
    bool m_OrionTakeover = false;
    bool m_ICPSSettled = false;
    bool m_MaxQAnnounced = false;

    bool m_AutopilotCircularize = true;
    bool m_ManualControlEnabled = false;
    bool m_RCSState = false;

    bool m_EnduranceMode = false;
    bool m_FireTriggered = false;
    bool m_DepressTriggered = false;
    bool m_ExplosionTriggered = false;

    bool m_RPressedLastFrame = false;
    bool m_SpacePressedLastFrame = false;
    bool m_CPressedLastFrame = false;
    bool m_MPressedLastFrame = false;

    double m_MissionTime = 0.0;
    double m_ICPSIgnitionTime = 0.0;
    double m_MaxQObserved = 0.0;
    double m_MaxQTime = 0.0;
    double m_TelemetryTimer = 0.0;
};

class DeepSpaceApp : public Mock::Application {
public:
    DeepSpaceApp() = default;

    int OnInitialize() override {
        PushLayer(new SimulationLayer());
        return 0;
    }

    void OnShutdown() override {
    }

    void PushLayer(Mock::Layer* layer) {
        m_Layers.push_back(std::unique_ptr<Mock::Layer>(layer));
    }

    void Run(Mock::Engine& engine) {
        for (auto& layer : m_Layers) {
            engine.PushLayer(std::move(layer));
        }
        engine.Run();
    }

private:
    std::vector<std::unique_ptr<Mock::Layer>> m_Layers;
};

}

DeepSpace::DeepSpaceApp* CreateDeepSpaceApp() {
    return new DeepSpace::DeepSpaceApp();
}
