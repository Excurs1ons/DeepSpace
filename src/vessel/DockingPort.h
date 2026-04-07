#pragma once
#include "../physics/PhysicsBody.h"
#include <memory>
#include <string>

namespace DeepSpace {

class Vessel;

enum class DockingState {
    Open,
    Approach,
    SoftCapture,
    HardDock
};

class DockingPort {
public:
    DockingPort(const std::string& name, const Vec3d& localPosition, const Vec3d& localDirection);
    
    const std::string& GetName() const { return m_Name; }
    const Vec3d& GetLocalPosition() const { return m_LocalPosition; }
    const Vec3d& GetLocalDirection() const { return m_LocalDirection; }
    DockingState GetState() const { return m_State; }
    
    bool CanInitiateSoftCapture(const Vec3d& incomingPosition, const Vec3d& incomingVelocity, const Vec3d& stationAngularVelocity);
    void InitiateSoftCapture();
    bool CanCompleteHardDock(double relativeVelocity);
    void CompleteHardDock();
    
    void SetDockedVessel(std::weak_ptr<Vessel> vessel) { m_DockedVessel = vessel; }
    std::weak_ptr<Vessel> GetDockedVessel() const { return m_DockedVessel; }
    
    void Undock();

private:
    std::string m_Name;
    Vec3d m_LocalPosition;
    Vec3d m_LocalDirection;
    DockingState m_State = DockingState::Open;
    std::weak_ptr<Vessel> m_DockedVessel;
    
    static constexpr double SOFT_CAPTURE_VELOCITY = 0.5;
    static constexpr double HARD_DOCK_VELOCITY = 0.1;
    static constexpr double HARD_DOCK_ANGLE_DEG = 5.0;
};

}