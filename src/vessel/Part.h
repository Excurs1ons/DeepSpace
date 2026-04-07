#pragma once
#include <algorithm>
#include <string>

namespace DeepSpace {

    class Part {
    public:
        Part(const std::string& name, double dryMass)
            : m_Name(name), m_DryMass(dryMass), m_Active(false), m_Stage(-1), m_Decoupled(false) {}

        virtual ~Part() = default;

        virtual void Update(double dt) {}

        virtual double GetMass() const {
            if (m_Decoupled) return 0.0;
            return m_DryMass;
        }

        virtual double GetThrust(double ambientPressure) const { return 0.0; }

        const std::string& GetName() const { return m_Name; }

        void SetActive(bool active) { m_Active = active; }
        bool IsActive() const { return m_Active && !m_Decoupled; }

        void SetStage(int stage) { m_Stage = stage; }
        int GetStage() const { return m_Stage; }

        void SetDecoupled(bool decoupled) { m_Decoupled = decoupled; }
        bool IsDecoupled() const { return m_Decoupled; }

    protected:
        std::string m_Name;
        double m_DryMass;
        bool m_Active;
        int m_Stage;
        bool m_Decoupled;
    };

    class DecouplerPart : public Part {
    public:
        DecouplerPart(const std::string& name, double mass)
            : Part(name, mass) {}

        void Activate() {
            SetActive(true);
        }
    };

    class EnginePart : public Part {
    public:
        EnginePart(const std::string& name, double mass, double maxThrustVac, double ispSL, double ispVac)
            : Part(name, mass), m_MaxThrustVac(maxThrustVac), m_IspSL(ispSL), m_IspVac(ispVac), m_Throttle(0.0) {}

        void SetThrottle(double throttle) {
            m_Throttle = std::max(0.0, std::min(1.0, throttle));
        }

        double GetThrottle() const { return m_Throttle; }

        double GetCurrentIsp(double ambientPressure) const {
            const double pSL = 101325.0;
            const double t = std::max(0.0, std::min(1.0, ambientPressure / pSL));
            return m_IspVac - (m_IspVac - m_IspSL) * t;
        }

        double GetThrust(double ambientPressure) const {
            if (!m_Active || m_Throttle <= 0.0) return 0.0;
            const double mDot = GetMaxMassFlowRate();
            return mDot * 9.80665 * GetCurrentIsp(ambientPressure) * m_Throttle;
        }

        double GetMaxMassFlowRate() const {
            return m_MaxThrustVac / (m_IspVac * 9.80665);
        }

        double GetCurrentMassFlowRate() const {
            if (!m_Active || m_Throttle <= 0.0) return 0.0;
            return GetMaxMassFlowRate() * m_Throttle;
        }

    private:
        double m_MaxThrustVac;
        double m_IspSL;
        double m_IspVac;
        double m_Throttle;
    };

    class FuelTankPart : public Part {
    public:
        FuelTankPart(const std::string& name, double dryMass, double fuelCapacity)
            : Part(name, dryMass), m_FuelCapacity(fuelCapacity), m_CurrentFuel(fuelCapacity) {}

        double GetMass() const override {
            if (m_Decoupled) return 0.0;
            return m_DryMass + m_CurrentFuel;
        }

        bool ConsumeFuel(double amount) {
            if (m_Decoupled || amount <= 0.0) return false;
            if (m_CurrentFuel >= amount) {
                m_CurrentFuel -= amount;
                return true;
            }
            m_CurrentFuel = 0.0;
            return false;
        }

        double GetCurrentFuel() const { return m_CurrentFuel; }

    private:
        double m_FuelCapacity;
        double m_CurrentFuel;
    };
}
