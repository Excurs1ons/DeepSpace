#include "EnduranceStation.h"

namespace DeepSpace {

EnduranceStation::EnduranceStation()
    : m_PhysicsBody()
    , m_Frame()
{
    m_PhysicsBody.SetMass(50000.0);
    m_PhysicsBody.SetInertia(1000000.0);
    
    m_DockingPorts.emplace_back("Port 1", Vec3d(RADIUS, 0, 0), Vec3d(1, 0, 0));
    m_DockingPorts.emplace_back("Port 2", Vec3d(-RADIUS, 0, 0), Vec3d(-1, 0, 0));
    m_DockingPorts.emplace_back("Port 3", Vec3d(0, RADIUS, 0), Vec3d(0, 1, 0));
    m_DockingPorts.emplace_back("Port 4", Vec3d(0, -RADIUS, 0), Vec3d(0, -1, 0));
    
    m_Modules[StationModuleId::Bridge] = {StationModuleId::Bridge, "Bridge", Vec3d(RADIUS, 0, 0), 50.0, CabinFire(), Depressurization()};
    m_Modules[StationModuleId::Lab] = {StationModuleId::Lab, "Lab", Vec3d(0, RADIUS, 0), 80.0, CabinFire(), Depressurization()};
    m_Modules[StationModuleId::Mess] = {StationModuleId::Mess, "Mess", Vec3d(-RADIUS, 0, 0), 60.0, CabinFire(), Depressurization()};
    m_Modules[StationModuleId::Sleep] = {StationModuleId::Sleep, "Sleep", Vec3d(0, -RADIUS, 0), 40.0, CabinFire(), Depressurization()};
    m_Modules[StationModuleId::Cargo] = {StationModuleId::Cargo, "Cargo", Vec3d(0, 0, 5), 200.0, CabinFire(), Depressurization()};
    m_Modules[StationModuleId::Airlock1] = {StationModuleId::Airlock1, "Airlock 1", Vec3d(RADIUS * 0.707, RADIUS * 0.707, 0), 10.0, CabinFire(), Depressurization()};
    m_Modules[StationModuleId::Airlock2] = {StationModuleId::Airlock2, "Airlock 2", Vec3d(-RADIUS * 0.707, RADIUS * 0.707, 0), 10.0, CabinFire(), Depressurization()};
    m_Modules[StationModuleId::Airlock3] = {StationModuleId::Airlock3, "Airlock 3", Vec3d(-RADIUS * 0.707, -RADIUS * 0.707, 0), 10.0, CabinFire(), Depressurization()};
    m_Modules[StationModuleId::Airlock4] = {StationModuleId::Airlock4, "Airlock 4", Vec3d(RADIUS * 0.707, -RADIUS * 0.707, 0), 10.0, CabinFire(), Depressurization()};
    
    SetSpinRate(NORMAL_RPM);
}

void EnduranceStation::SetPosition(const Vec3d& position) {
    m_Frame.SetOrigin(position);
    m_PhysicsBody.SetPosition(position);
}

void EnduranceStation::SetSpinRate(double rpm) {
    m_Frame.SetAngularVelocityRpm(rpm);
}

void EnduranceStation::EmergencySpinUp() {
    SetSpinRate(EMERGENCY_RPM);
}

Vec3d EnduranceStation::GetModuleWorldPosition(StationModuleId moduleId) const {
    auto it = m_Modules.find(moduleId);
    if (it != m_Modules.end()) {
        return m_Frame.ToInertial(it->second.localPosition, 0.0);
    }
    return m_Frame.GetOrigin();
}

double EnduranceStation::GetArtificialGravityAtModule(StationModuleId moduleId) const {
    auto it = m_Modules.find(moduleId);
    if (it != m_Modules.end()) {
        Vec3d ag = m_Frame.GetArtificialGravity(it->second.localPosition);
        return ag.Length();
    }
    return 0.0;
}

void EnduranceStation::TriggerFire(StationModuleId module) {
    auto it = m_Modules.find(module);
    if (it != m_Modules.end()) {
        it->second.fire.Ignite();
    }
}

void EnduranceStation::TriggerDepressurization(StationModuleId module, double leakArea) {
    auto it = m_Modules.find(module);
    if (it != m_Modules.end()) {
        it->second.depressurization.CreateLeak(leakArea);
    }
}

void EnduranceStation::TriggerExplosion(StationModuleId airlock) {
    auto it = m_Modules.find(airlock);
    if (it != m_Modules.end()) {
        Vec3d explosionPos = it->second.localPosition;
        
        m_Explosion.TriggerExplosion(explosionPos, 1000000.0);
        m_ExplosionTriggered = true;
        
        m_Depressurization.CreateLeak(0.5);
        
        Vec3d torque = m_Explosion.GetTorqueFromAsymmetricDamage();
        m_PhysicsBody.AddTorque3D(torque);
        
        MOCK_INFO("EXPLOSION at %s! Torque applied: (%.1f, %.1f, %.1f)", 
                  it->second.name.c_str(), torque.x, torque.y, torque.z);
    }
}

void EnduranceStation::Update(double dt) {
    for (auto& pair : m_Modules) {
        pair.second.fire.Update(dt);
        pair.second.depressurization.Update(dt);
    }
    
    if (m_ExplosionTriggered) {
        m_Explosion.Update(dt);
        
        double overpressure = m_Explosion.GetOverpressureAt(m_PhysicsBody.GetPosition(), 0.0);
        MOCK_TRACE("[Explosion] Overpressure: %.2f Pa", overpressure);
    }
    
    UpdateDocking(dt);
}

bool EnduranceStation::InitiateDocking(const std::shared_ptr<Vessel>& vessel, const Vec3d& approachPosition, const Vec3d& approachVelocity) {
    if (m_IsDockingInProgress || m_IsDocked) return false;
    
    const Vec3d& stationVel = m_PhysicsBody.GetVelocity();
    const Vec3d relVel = approachVelocity - stationVel;
    const double relSpeed = relVel.Length();
    
    if (relSpeed > 0.5) return false;
    
    m_ApproachingVessel = vessel;
    m_IsDockingInProgress = true;
    m_DockingProgress = 0.0;
    
    MOCK_INFO("Docking initiated with relative velocity: %.2f m/s", relSpeed);
    return true;
}

void EnduranceStation::UpdateDocking(double dt) {
    if (!m_IsDockingInProgress) return;
    
    m_DockingProgress += dt * 0.1;
    
    if (m_DockingProgress >= 1.0) {
        m_IsDockingInProgress = false;
        m_IsDocked = true;
        m_DockedVessel = m_ApproachingVessel;
        MOCK_INFO("Docking complete!");
    }
}

void EnduranceStation::Undock() {
    m_IsDocked = false;
    m_ApproachingVessel.reset();
    m_DockingProgress = 0.0;
}

}
