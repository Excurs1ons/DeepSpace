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
        it->second.depressurization.CreateLeak(1.0);
    }
}

void EnduranceStation::Update(double dt) {
    for (auto& pair : m_Modules) {
        pair.second.fire.Update(dt);
        pair.second.depressurization.Update(dt);
    }
}

}
