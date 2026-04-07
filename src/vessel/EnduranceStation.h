#pragma once
#include "../physics/PhysicsBody.h"
#include "../physics/RotatingFrame.h"
#include "DockingPort.h"
#include "Vessel.h"
#include "../environment/CabinFire.h"
#include "../environment/Depressurization.h"
#include "../environment/AirlockExplosion.h"
#include <vector>
#include <map>
#include <string>

namespace DeepSpace {

enum class StationModuleId { Bridge, Lab, Mess, Sleep, Cargo, Airlock1, Airlock2, Airlock3, Airlock4 };

struct StationModule {
    StationModuleId id;
    std::string name;
    Vec3d localPosition;
    double volume;
    CabinFire fire;
    Depressurization depressurization;
};

class EnduranceStation {
public:
    EnduranceStation();
    
    void SetPosition(const Vec3d& position);
    void SetSpinRate(double rpm);
    void EmergencySpinUp();
    
    const Vec3d& GetPosition() const { return m_Frame.GetOrigin(); }
    double GetSpinRateRpm() const { return m_Frame.GetAngularVelocityRpm(); }
    const Vec3d& GetAngularVelocity() const { return m_Frame.GetAngularVelocity(); }
    
    RotatingFrame& GetRotatingFrame() { return m_Frame; }
    const RotatingFrame& GetRotatingFrame() const { return m_Frame; }
    
    PhysicsBody& GetPhysicsBody() { return m_PhysicsBody; }
    const PhysicsBody& GetPhysicsBody() const { return m_PhysicsBody; }
    
    std::vector<DockingPort>& GetDockingPorts() { return m_DockingPorts; }
    const std::vector<DockingPort>& GetDockingPorts() const { return m_DockingPorts; }
    
    Vec3d GetModuleWorldPosition(StationModuleId moduleId) const;
    double GetArtificialGravityAtModule(StationModuleId moduleId) const;
    
    void TriggerFire(StationModuleId module);
    void TriggerDepressurization(StationModuleId module, double leakArea);
    void TriggerExplosion(StationModuleId airlock);
    
    void Update(double dt);
    
    bool InitiateDocking(const std::shared_ptr<Vessel>& vessel, const Vec3d& approachPosition, const Vec3d& approachVelocity);
    void UpdateDocking(double dt);
    bool IsDocked() const { return m_IsDocked; }
    std::weak_ptr<Vessel> GetDockedVessel() const { return m_DockedVessel; }
    void Undock();
    
    StationModule& GetModule(StationModuleId id) { return m_Modules[id]; }
    
    static constexpr double RADIUS = 40.0;
    static constexpr double NORMAL_RPM = 5.6;
    static constexpr double EMERGENCY_RPM = 68.0;

private:
    PhysicsBody m_PhysicsBody;
    RotatingFrame m_Frame;
    std::vector<DockingPort> m_DockingPorts;
    std::map<StationModuleId, StationModule> m_Modules;
    AirlockExplosion m_Explosion;
    bool m_ExplosionTriggered = false;
    Depressurization m_Depressurization;
    
    bool m_IsDocked = false;
    std::shared_ptr<Vessel> m_ApproachingVessel;
    std::weak_ptr<Vessel> m_DockedVessel;
    double m_DockingProgress = 0.0;
    bool m_IsDockingInProgress = false;
};

}
