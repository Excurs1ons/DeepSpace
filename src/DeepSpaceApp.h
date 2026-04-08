#pragma once
#include "engine/MockEngine.h"
#include "environment/Planet.h"
#include "physics/Aerodynamics.h"
#include "physics/OrbitalElements.h"
#include "vessel/EnduranceStation.h"
#include "vessel/PartLibrary.h"
#include "vessel/Vessel.h"
#include "environment/ThermalSimulation.h"
#include "environment/DamageSystem.h"
#include "mission/MissionControl.h"
#include "mission/Artemis2Mission.h"
#include "mission/MissionConfig.h"

namespace DeepSpace {
using Vec3d = Mock::Vec3d;

class SimulationLayer : public Mock::Layer {
public:
    SimulationLayer(bool headless = false, const std::string& configPath = "missions/artemis2.conf")
        : Layer("SimulationLayer"),
          m_Earth("Earth", 5.9722e24, 6371000.0, Atmosphere(101325.0, 8500.0)),
          m_Vessel(std::make_shared<Vessel>("Artemis II Mission")),
          m_MissionControl(*m_Vessel, m_Earth),
          m_EnduranceStation(nullptr),
          m_EnduranceMode(false),
          m_FireTriggered(false),
          m_DepressTriggered(false),
          m_ExplosionTriggered(false),
          m_ThermalSimulation(),
          m_HeadlessMode(headless),
          m_MissionComplete(false),
          m_Initialized(false) {
        try {
            m_Config = MissionConfig::Load(configPath);
            MOCK_INFO("Config loaded from: %s", configPath.c_str());
        } catch (const std::exception& e) {
            MOCK_WARN("Failed to load config: %s, using defaults", e.what());
        }
        m_Vessel->RecalculateMass();
    }

    void OnAttach() override {
        if (m_Initialized) return;
        m_Initialized = true;
        
        MOCK_INFO("Simulation Layer Attached");
        
        if (m_HeadlessMode) {
            m_MissionControl.LoadMission(Artemis2Mission::CreateDefaultMission());
        }
        
        BuildArtemis2FlightPlan();
        m_Vessel->RecalculateMass();
        
        auto& body = m_Vessel->GetPhysicsBody();
        body.SetPosition({0.0, m_Earth.GetRadius(), 0.0});
        body.SetOrientation({0.0, 1.0, 0.0});
        body.SetInertia(12000000.0);
        
        m_Vessel->ActivateNextStage();
        MOCK_INFO("T-0: Artemis II ascent stage ignition");
        MOCK_INFO("Vessel mass: %.0f kg", m_Vessel->GetMass());
    }

    void OnUpdate(double dt) override {
        if (dt <= 0.0) return;

        PhysicsBody& body = m_Vessel->GetPhysicsBody();
        const Vec3d pos = body.GetPosition();
        const Vec3d vel = body.GetVelocity();
        const double altitude = m_Earth.GetAltitude(pos);
        const double ambientPressure = m_Earth.GetAtmosphere().GetPressure(altitude);
        const double speed = vel.Length();
        const double density = m_Earth.GetAtmosphere().GetDensity(altitude);

        body.AddForce(m_Earth.GetGravityAt(pos) * body.GetMass());

        m_ThermalSimulation.Update(dt, speed, density, 1.0 - m_Vessel->GetTotalDamage());
        
        Aerodynamics::ApplyAerodynamics(body, m_Earth.GetAtmosphere(), altitude, 
            m_Vessel->GetTotalDamage() * 0.2);

        m_MissionTime += dt;
        
        const double dynamicPressure = 0.5 * density * speed * speed;
        
        m_DamageSystem.Update(dt, *m_Vessel);

        if (!m_ManualControlEnabled) {
            ApplyArtemisAscentGuidance(altitude, body);
        }

        ManageMissionEvents(altitude, dynamicPressure);

        const EngineStatus status = m_Vessel->Update(dt, ambientPressure);
        m_Vessel->UpdateWithDamage(dt, ambientPressure);
        body.Update(dt);

        const auto elements = OrbitalMechanics::CalculateElements(body.GetPosition(), body.GetVelocity(), m_Earth);

        if (m_AutopilotCircularize && m_BoosterSeparated) {
            ApplyCircularizationGuidance(elements, altitude, body);
        }

        if (m_HeadlessMode) {
            m_MissionControl.Update(dt, status);
            if (m_MissionControl.GetOutcome() != MissionOutcome::IN_PROGRESS) {
                m_MissionComplete = true;
                FinalizeMission();
            }
        } else {
            HandleInput(dt);
            AutoVerifyDamageSystem(dt);
        }

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
        MOCK_INFO("BuildArtemis2FlightPlan: ICPS thrust=%.0f N, Vacuum Isp=%.0f s, Merlin thrust=%.0f N", 
            m_Config.rl10.thrust_N, m_Config.rl10.vacuumIsp_s, m_Config.merlin.thrust_N);
        
        auto orionMmh = PartLibrary::CreateArtemis2OrionMMHTank(
            m_Config.orionMMH.dryMass_kg, m_Config.orionMMH.fuelMass_kg);
        auto orionNto = PartLibrary::CreateArtemis2OrionNTOTank(
            m_Config.orionNTO.dryMass_kg, m_Config.orionNTO.fuelMass_kg);
        auto orionAj10 = PartLibrary::CreateAJ10_190(
            m_Config.aj10.thrust_N,
            m_Config.aj10.seaLevelIsp_s,
            m_Config.aj10.vacuumIsp_s,
            m_Config.aj10.OF_ratio);
        orionMmh->SetStage(2);
        orionNto->SetStage(2);
        orionAj10->SetStage(2);

        auto icpsLh2 = PartLibrary::CreateArtemis2ICPSLH2Tank(
            m_Config.icpsLH2.dryMass_kg, m_Config.icpsLH2.fuelMass_kg);
        auto icpsLox = PartLibrary::CreateArtemis2ICPSLOXTank(
            m_Config.icpsLOX.dryMass_kg, m_Config.icpsLOX.fuelMass_kg);
        auto icpsEngine = PartLibrary::CreateRL10B2(
            m_Config.rl10.thrust_N,
            m_Config.rl10.seaLevelIsp_s,
            m_Config.rl10.vacuumIsp_s,
            m_Config.rl10.OF_ratio);
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

        auto coreRp1 = PartLibrary::CreateFalcon9S1RP1Tank(
            m_Config.coreRP1.dryMass_kg, m_Config.coreRP1.fuelMass_kg);
        auto coreLox = PartLibrary::CreateFalcon9S1LOXTank(
            m_Config.coreLOX.dryMass_kg, m_Config.coreLOX.fuelMass_kg);
        coreRp1->SetStage(0);
        coreLox->SetStage(0);
        m_Vessel->AddPart(coreRp1);
        m_Vessel->AddPart(coreLox);
        m_BoosterCoreLoxTank = coreLox;

        for (int i = 0; i < 13; ++i) {
            auto engine = PartLibrary::CreateMerlin1D(
                m_Config.merlin.thrust_N,
                m_Config.merlin.seaLevelIsp_s,
                m_Config.merlin.vacuumIsp_s,
                m_Config.merlin.OF_ratio);
            engine->SetStage(0);
            m_Vessel->AddPart(engine);
        }

        MOCK_INFO("Artemis II stack ready: %s, mass=%.0fkg", m_Vessel->GetName().c_str(), m_Vessel->GetPhysicsBody().GetMass());
    }

    void HandleInput(double dt) {
        auto input = Mock::InputManager::Get();
        
        char cmd = input.GetCharInput();
        if (cmd != 0) {
            ProcessCommand(cmd);
        }
        
        if (input.IsKeyJustPressed(Mock::KeyCode::T)) {
            m_DamageSystem.TriggerDamage(DamageType::TPS, 0.3, *m_Vessel);
            MOCK_WARN("DAMAGE: TPS impact! Thermal protection compromised!");
        }
        if (input.IsKeyJustPressed(Mock::KeyCode::S)) {
            m_DamageSystem.TriggerDamage(DamageType::STRUCTURAL, 0.2, *m_Vessel);
            MOCK_WARN("DAMAGE: Structural damage! Hull integrity reduced!");
        }
        if (input.IsKeyJustPressed(Mock::KeyCode::P)) {
            m_DamageSystem.TriggerDamage(DamageType::PROPULSION, 0.25, *m_Vessel);
            MOCK_WARN("DAMAGE: Propulsion hit! Engine performance degraded!");
        }
        if (input.IsKeyJustPressed(Mock::KeyCode::L)) {
            m_DamageSystem.TriggerDamage(DamageType::LIFESUPPORT, 0.15, *m_Vessel);
            MOCK_WARN("DAMAGE: Life support damaged! Cabin systems failing!");
        }

        const bool rPressed = input.IsKeyPressed(Mock::KeyCode::R);
        const bool spacePressed = input.IsKeyPressed(Mock::KeyCode::Space);
        const bool cPressed = input.IsKeyPressed(Mock::KeyCode::C);
        const bool mPressed = input.IsKeyPressed(Mock::KeyCode::M);

        if (rPressed && !m_RPressedLastFrame) {
            if (m_FireTriggered || m_DepressTriggered || m_ExplosionTriggered) {
                m_FireTriggered = false;
                m_DepressTriggered = false;
                m_ExplosionTriggered = false;
                if (m_EnduranceStation) {
                    m_EnduranceStation->SetSpinRate(EnduranceStation::NORMAL_RPM);
                }
                MOCK_INFO("Emergency systems reset");
            } else {
                m_RCSState = !m_RCSState;
                m_Vessel->GetRCS().SetEnabled(m_RCSState);
                MOCK_INFO("RCS %s", m_RCSState ? "ON" : "OFF");
            }
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

        if (input.IsKeyPressed(Mock::KeyCode::T) && !m_TPressedLastFrame) {
            m_DamageSystem.TriggerDamage(DamageType::TPS, 0.3, *m_Vessel);
            MOCK_WARN("DAMAGE: TPS impact! Thermal protection compromised!");
        }
        if (input.IsKeyPressed(Mock::KeyCode::S) && !m_SDamagePressedLastFrame) {
            m_DamageSystem.TriggerDamage(DamageType::STRUCTURAL, 0.2, *m_Vessel);
            MOCK_WARN("DAMAGE: Structural damage! Hull integrity reduced!");
        }
        if (input.IsKeyPressed(Mock::KeyCode::P) && !m_PDamagePressedLastFrame) {
            m_DamageSystem.TriggerDamage(DamageType::PROPULSION, 0.25, *m_Vessel);
            MOCK_WARN("DAMAGE: Propulsion hit! Engine performance degraded!");
        }
        if (input.IsKeyPressed(Mock::KeyCode::L) && !m_LDamagePressedLastFrame) {
            m_DamageSystem.TriggerDamage(DamageType::LIFESUPPORT, 0.15, *m_Vessel);
            MOCK_WARN("DAMAGE: Life support damaged! Cabin systems failing!");
        }

        m_RPressedLastFrame = rPressed;
        m_SpacePressedLastFrame = spacePressed;
        m_CPressedLastFrame = cPressed;
        m_MPressedLastFrame = mPressed;
        m_TPressedLastFrame = input.IsKeyPressed(Mock::KeyCode::T);
        m_SDamagePressedLastFrame = input.IsKeyPressed(Mock::KeyCode::S);
        m_PDamagePressedLastFrame = input.IsKeyPressed(Mock::KeyCode::P);
        m_LDamagePressedLastFrame = input.IsKeyPressed(Mock::KeyCode::L);

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

    void AutoVerifyDamageSystem(double dt) {
        static bool tpsTriggered = false;
        static bool structTriggered = false;
        static bool propTriggered = false;
        static bool lifeTriggered = false;
        static bool repairTriggered = false;
        static double lastTPS = 0.0;
        static double lastStruct = 0.0;
        
        if (!tpsTriggered && m_MissionTime >= 5.0) {
            tpsTriggered = true;
            m_DamageSystem.TriggerDamage(DamageType::TPS, 0.3, *m_Vessel);
            MOCK_INFO("VERIFY: TPS damage 30%% applied at T=%.1fs", m_MissionTime);
        }
        if (!structTriggered && m_MissionTime >= 10.0) {
            structTriggered = true;
            m_DamageSystem.TriggerDamage(DamageType::STRUCTURAL, 0.2, *m_Vessel);
            MOCK_INFO("VERIFY: Structural damage 20%% applied at T=%.1fs", m_MissionTime);
        }
        if (!propTriggered && m_MissionTime >= 15.0) {
            propTriggered = true;
            m_DamageSystem.TriggerDamage(DamageType::PROPULSION, 0.25, *m_Vessel);
            MOCK_INFO("VERIFY: Propulsion damage 25%% applied at T=%.1fs", m_MissionTime);
        }
        if (!lifeTriggered && m_MissionTime >= 20.0) {
            lifeTriggered = true;
            m_DamageSystem.TriggerDamage(DamageType::LIFESUPPORT, 0.15, *m_Vessel);
            MOCK_INFO("VERIFY: Life support damage 15%% applied at T=%.1fs", m_MissionTime);
        }
        if (!repairTriggered && m_MissionTime >= 25.0) {
            repairTriggered = true;
            m_DamageSystem.ApplyRepair(DamageType::TPS, 0.3);
            m_DamageSystem.ApplyRepair(DamageType::STRUCTURAL, 0.2);
            MOCK_INFO("VERIFY: Repair applied at T=%.1fs", m_MissionTime);
        }
        
        double tps = m_DamageSystem.GetDamageLevel(DamageType::TPS);
        double strukt = m_DamageSystem.GetDamageLevel(DamageType::STRUCTURAL);
        
        if (tps > lastTPS && lastTPS > 0) {
            MOCK_INFO("VERIFY: TPS cascade triggered! %.0f%% -> %.0f%%", lastTPS*100, tps*100);
        }
        if (strukt > lastStruct && lastStruct > 0) {
            MOCK_INFO("VERIFY: Structural cascade triggered! %.0f%% -> %.0f%%", lastStruct*100, strukt*100);
        }
        
        lastTPS = tps;
        lastStruct = strukt;
    }

    void ProcessCommand(char cmd) {
        switch (cmd) {
            case 't':
            case 'T':
                m_DamageSystem.TriggerDamage(DamageType::TPS, 0.3, *m_Vessel);
                MOCK_WARN("DAMAGE: TPS impact! Thermal protection compromised!");
                break;
            case 's':
            case 'S':
                m_DamageSystem.TriggerDamage(DamageType::STRUCTURAL, 0.2, *m_Vessel);
                MOCK_WARN("DAMAGE: Structural damage! Hull integrity reduced!");
                break;
            case 'p':
            case 'P':
                m_DamageSystem.TriggerDamage(DamageType::PROPULSION, 0.25, *m_Vessel);
                MOCK_WARN("DAMAGE: Propulsion hit! Engine performance degraded!");
                break;
            case 'l':
            case 'L':
                m_DamageSystem.TriggerDamage(DamageType::LIFESUPPORT, 0.15, *m_Vessel);
                MOCK_WARN("DAMAGE: Life support damaged! Cabin systems failing!");
                break;
            case 'r':
            case 'R':
                m_DamageSystem.ApplyRepair(DamageType::TPS, 0.5);
                m_DamageSystem.ApplyRepair(DamageType::STRUCTURAL, 0.5);
                m_DamageSystem.ApplyRepair(DamageType::PROPULSION, 0.5);
                m_DamageSystem.ApplyRepair(DamageType::LIFESUPPORT, 0.5);
                MOCK_INFO("REPAIR: All systems partially restored");
                break;
        }
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
            MOCK_INFO("Booster/core depletion - staging to ICPS (LOX=%.0fkg, time=%.1fs)", 
                m_BoosterCoreLoxTank->GetCurrentFuel(), m_MissionTime);
            m_Vessel->ActivateNextStage();
            m_BoosterSeparated = true;
            m_CircularizingBurnStarted = false;
            m_Vessel->GetRCS().SetEnabled(true);
            m_RCSState = true;
            m_ICPSIgnitionTime = m_MissionTime;
        }

        if (m_BoosterSeparated && !m_ICPSSettled) {
            m_ICPSSettled = true;
            MOCK_INFO("ICPS guidance active at T+%.1fs", m_MissionTime);
        }

        if (m_BoosterSeparated && !m_TEIWindowOpened && altitude > 185000.0) {
            m_TEIWindowOpened = true;
            MOCK_INFO("Artemis II translunar preparation window opened");
        }
    }

    void FinalizeMission() {
        const auto& summary = m_MissionControl.GetSummary();
        const auto outcome = m_MissionControl.GetOutcome();
        
        MOCK_INFO("========================================");
        MOCK_INFO("MISSION %s", outcome == MissionOutcome::SUCCESS ? "SUCCESS" : 
                          outcome == MissionOutcome::FAILURE ? "FAILED" :
                          outcome == MissionOutcome::ABORT ? "ABORTED" : "TIMEOUT");
        MOCK_INFO("========================================");
        MOCK_INFO("Duration: %.1f seconds", summary.duration_s);
        MOCK_INFO("Final Orbit: Ap=%.0fkm, Pe=%.0fkm", 
                  summary.finalOrbit.apoapsis_m / 1000.0,
                  summary.finalOrbit.periapsis_m / 1000.0);
        MOCK_INFO("Target Orbit: Ap=%.0fkm, Pe=%.0fkm",
                  summary.targetOrbit.apoapsis_m / 1000.0,
                  summary.targetOrbit.periapsis_m / 1000.0);
        MOCK_INFO("Max-Q: %.0f Pa at %.0fm altitude", summary.maxQ_pa, summary.maxQAltitude_m);
        MOCK_INFO("Events triggered: %zu", summary.allEvents.size());
        
        m_MissionControl.ExportCSV("artemis2_telemetry.csv");
        m_MissionControl.ExportSummary("artemis2_summary.json");
        MOCK_INFO("Telemetry exported to artemis2_telemetry.csv and artemis2_summary.json");
    }

    void ApplyArtemisAscentGuidance(double altitude, PhysicsBody& body) {
        const double pitchStartAlt = m_Config.guidance.pitchStartAlt_m;
        const double pitchEndAlt = m_Config.guidance.pitchEndAlt_m;
        
        if (altitude <= pitchStartAlt || altitude >= pitchEndAlt || m_BoosterSeparated) {
            return;
        }

        const double pitchFactor = std::pow((altitude - pitchStartAlt) / (pitchEndAlt - pitchStartAlt), 0.7);
        const double targetAngle = pitchFactor * (M_PI / 2.0);
        
        body.SetOrientation({std::sin(targetAngle), std::cos(targetAngle), 0.0});

        if (altitude > 9500.0 && altitude < 15000.0) {
            m_Vessel->SetStageThrottle(2, 0.72);
        } else {
            m_Vessel->SetStageThrottle(2, 1.0);
        }
    }

    void ApplyCircularizationGuidance(const OrbitalElements& orbit, double altitude, PhysicsBody& body) {
        if (!m_BoosterSeparated) return;
        
        const Vec3d velocity = body.GetVelocity();
        const Vec3d prograde = velocity.Normalized();
        
        const double targetPe = m_Config.targetPe_km * 1000.0;
        const double tolerance = 5000.0;
        
        const double actualPe = orbit.periapsis > 0 ? orbit.periapsis : altitude;
        const bool peInRange = actualPe >= targetPe - tolerance;
        
        if (peInRange) {
            m_Vessel->SetStageThrottle(1, 0.0);
            if (!m_OrbitAchieved) {
                m_OrbitAchieved = true;
                MOCK_INFO("GUIDANCE: CIRCULARIZATION COMPLETE! Pe=%.0fkm",
                    actualPe / 1000.0);
            }
            return;
        }
        
        body.SetOrientation(prograde);
        m_Vessel->SetStageThrottle(1, 1.0);
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

        MOCK_INFO(
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
    
    MissionControl m_MissionControl;

    bool m_BoosterSeparated = false;
    bool m_TEIWindowOpened = false;
    bool m_OrionTakeover = false;
    bool m_ICPSSettled = false;
    bool m_MaxQAnnounced = false;
    bool m_OrbitAchieved = false;
    bool m_CircularizingBurnStarted = false;

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
    bool m_TPressedLastFrame = false;
    bool m_SDamagePressedLastFrame = false;
    bool m_PDamagePressedLastFrame = false;
    bool m_LDamagePressedLastFrame = false;

    double m_MissionTime = 0.0;
    double m_ICPSIgnitionTime = 0.0;
    double m_MaxQObserved = 0.0;
    double m_MaxQTime = 0.0;
    double m_TelemetryTimer = 0.0;
    
    bool m_HeadlessMode = false;
    bool m_MissionComplete = false;
    bool m_Initialized = false;
    MissionConfig m_Config;
    
public:
    bool IsMissionComplete() const { return m_MissionComplete; }
    double GetCurrentAltitude() const {
        return m_Earth.GetAltitude(m_Vessel->GetPhysicsBody().GetPosition());
    }
    double GetCurrentVelocity() const {
        return m_Vessel->GetPhysicsBody().GetVelocity().Length();
    }
    double GetVesselMass() const {
        return m_Vessel->GetMass();
    }
    double GetTotalThrust() const {
        double thrust = 0.0;
        for (const auto& part : m_Vessel->GetParts()) {
            auto* engine = dynamic_cast<EnginePart*>(part.get());
            if (engine && engine->IsActive() && engine->GetThrottle() > 0.0) {
                double ambient = m_Earth.GetAtmosphere().GetPressure(GetCurrentAltitude());
                thrust += engine->GetThrust(ambient);
            }
        }
        return thrust;
    }
    Vec3d GetPosition() const {
        return m_Vessel->GetPhysicsBody().GetPosition();
    }
    Vec3d GetVelocity() const {
        return m_Vessel->GetPhysicsBody().GetVelocity();
    }
    double GetNetForce() const {
        Vec3d force = m_Earth.GetGravityAt(GetPosition()) * GetVesselMass();
        force += m_Vessel->GetPhysicsBody().GetAccumulatedForce();
        return force.y;
    }
    
private:

    ThermalSimulation m_ThermalSimulation;
    
    DamageSystem m_DamageSystem;
};

class DeepSpaceApp : public Mock::Application {
public:
    explicit DeepSpaceApp(bool headless = false) : m_HeadlessDefault(headless) {}

    int OnInitialize() override {
        m_SimulationLayer = new SimulationLayer(m_HeadlessDefault);
        PushLayer(m_SimulationLayer);
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
        if (m_HeadlessDefault) {
            RunHeadless(engine);
        } else {
            engine.Run();
        }
    }
    
    void RunHeadless(Mock::Engine& engine) {
        if (!m_SimulationLayer) {
            MOCK_ERROR("RunHeadless: SimulationLayer not found!");
            return;
        }
        
        MOCK_INFO("RunHeadless: Initializing simulation layer...");
        m_SimulationLayer->OnAttach();
        
        const double fixedDt = 0.1;
        double totalSimTime = 0.0;
        const double maxSimTime = 7200.0;
        double lastReportTime = 0.0;
        const double reportInterval = 10.0;
        
        MOCK_INFO("RunHeadless: Starting simulation (max %.0fs, dt=%.1fs)", maxSimTime, fixedDt);
        
        int loopCount = 0;
        bool running = true;
        while (running && totalSimTime < maxSimTime) {
            totalSimTime += fixedDt;
            loopCount++;
            
            m_SimulationLayer->OnUpdate(fixedDt);
            
            if (loopCount <= 3) {
                double altitude = m_SimulationLayer->GetCurrentAltitude();
                double velocity = m_SimulationLayer->GetCurrentVelocity();
                MOCK_TRACE("Headless T=%.1fs: Alt=%.0fm Vel=%.0fm/s", totalSimTime, altitude, velocity);
            }
            
            if (totalSimTime - lastReportTime >= reportInterval) {
                lastReportTime = totalSimTime;
                double altitude = m_SimulationLayer->GetCurrentAltitude();
                double velocity = m_SimulationLayer->GetCurrentVelocity();
                MOCK_INFO("[T=%.0fs] Alt=%.0fm Vel=%.0fm/s", totalSimTime, altitude, velocity);
            }
            
            if (m_SimulationLayer->IsMissionComplete()) {
                MOCK_INFO("RunHeadless: Mission complete at T=%.1fs", totalSimTime);
                running = false;
            }
        }
        
        if (totalSimTime >= maxSimTime) {
            MOCK_INFO("RunHeadless: Max simulation time reached (%.1fs)", totalSimTime);
        }
    }

private:
    std::vector<std::unique_ptr<Mock::Layer>> m_Layers;
    SimulationLayer* m_SimulationLayer = nullptr;
    bool m_HeadlessDefault = false;
};

}

DeepSpace::DeepSpaceApp* CreateDeepSpaceApp() {
    return new DeepSpace::DeepSpaceApp();
}
