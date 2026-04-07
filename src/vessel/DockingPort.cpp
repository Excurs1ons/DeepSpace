#include "DockingPort.h"
#include "Vessel.h"
#include <cmath>

namespace DeepSpace {

DockingPort::DockingPort(const std::string& name, const Vec3d& localPosition, const Vec3d& localDirection)
    : m_Name(name)
    , m_LocalPosition(localPosition)
    , m_LocalDirection(localDirection.Normalized())
{}

bool DockingPort::CanInitiateSoftCapture(const Vec3d& incomingPosition, const Vec3d& incomingVelocity, const Vec3d& stationAngularVelocity) {
    if (m_State != DockingState::Open) return false;
    
    Vec3d relVel = incomingVelocity - stationAngularVelocity;
    double speed = relVel.Length();
    
    if (speed > SOFT_CAPTURE_VELOCITY) return false;
    
    Vec3d toPort = m_LocalPosition - incomingPosition;
    double dist = toPort.Length();
    if (dist > 10.0) return false;
    
    return true;
}

void DockingPort::InitiateSoftCapture() {
    if (m_State == DockingState::Open) {
        m_State = DockingState::SoftCapture;
    }
}

bool DockingPort::CanCompleteHardDock(double relativeVelocity) {
    if (m_State != DockingState::SoftCapture) return false;
    return relativeVelocity < HARD_DOCK_VELOCITY;
}

void DockingPort::CompleteHardDock() {
    if (m_State == DockingState::SoftCapture) {
        m_State = DockingState::HardDock;
    }
}

void DockingPort::Undock() {
    m_State = DockingState::Open;
    m_DockedVessel.reset();
}

}