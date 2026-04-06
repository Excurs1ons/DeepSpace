#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <cmath>

namespace Prisma {

    // --- Math ---
    struct Vec2 { float x, y; };
    struct Vec3 { float x, y, z; };
    
    // Double precision vector for orbital mechanics
    struct Vec3d { 
        double x, y, z; 
        
        Vec3d() : x(0), y(0), z(0) {}
        Vec3d(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
        
        Vec3d operator+(const Vec3d& other) const { return {x + other.x, y + other.y, z + other.z}; }
        Vec3d operator-(const Vec3d& other) const { return {x - other.x, y - other.y, z - other.z}; }
        Vec3d operator*(double scalar) const { return {x * scalar, y * scalar, z * scalar}; }
        Vec3d operator/(double scalar) const { return {x / scalar, y / scalar, z / scalar}; }
        Vec3d& operator+=(const Vec3d& other) { x += other.x; y += other.y; z += other.z; return *this; }
        Vec3d& operator-=(const Vec3d& other) { x -= other.x; y -= other.y; z -= other.z; return *this; }
        
        static double Dot(const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
        static Vec3d Cross(const Vec3d& a, const Vec3d& b) {
            return {
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
            };
        }

        double Length() const { return std::sqrt(x*x + y*y + z*z); }
        Vec3d Normalized() const { 
            double len = Length(); 
            if (len > 0) return *this / len; 
            return *this; 
        }
    };

    // --- Events ---
    class Event {
    public:
        virtual ~Event() = default;
        virtual std::string ToString() const { return "Event"; }
    };

    // --- Timestep ---
    class Timestep {
    public:
        Timestep(double time = 0.0) : m_Time(time) {}
        double GetSeconds() const { return m_Time; }
        double GetMilliseconds() const { return m_Time * 1000.0; }
    private:
        double m_Time;
    };

    // --- Layer System ---
    class Layer {
    public:
        Layer(const std::string& name = "Layer") : m_DebugName(name) {}
        virtual ~Layer() = default;

        virtual void OnAttach() {}
        virtual void OnDetach() {}
        virtual void OnUpdate(Timestep ts) {}
        virtual void OnEvent(Event& event) {}

        const std::string& GetName() const { return m_DebugName; }
    protected:
        std::string m_DebugName;
    };

    class LayerStack {
    public:
        LayerStack() = default;
        ~LayerStack() {
            for (Layer* layer : m_Layers) {
                layer->OnDetach();
                delete layer;
            }
        }

        void PushLayer(Layer* layer) {
            m_Layers.emplace_back(layer);
            layer->OnAttach();
        }

        std::vector<Layer*>::iterator begin() { return m_Layers.begin(); }
        std::vector<Layer*>::iterator end() { return m_Layers.end(); }
    private:
        std::vector<Layer*> m_Layers;
    };

    // --- Input ---
    namespace Key {
        enum {
            W = 87, S = 83, A = 65, D = 68, 
            Q = 81, E = 69, 
            Space = 32, Shift = 16, Ctrl = 17,
            X = 88, Z = 90
        };
    }

    class Input {
    public:
        static bool IsKeyPressed(int keycode) { return false; } // Mocked
        static bool IsMouseButtonPressed(int button) { return false; } // Mocked
        static Vec2 GetMousePosition() { return { 0.0f, 0.0f }; }
    };

    // --- Application ---
    class Application {
    public:
        Application() = default;
        virtual ~Application() = default;

        virtual int OnInitialize() = 0;
        virtual void OnUpdate(Timestep ts) {
            for (auto layer : m_LayerStack)
                layer->OnUpdate(ts);
        }
        virtual void OnRender() {}
        virtual void OnEvent(Event& e) {
            for (auto it = m_LayerStack.end(); it != m_LayerStack.begin();) {
                (*--it)->OnEvent(e);
            }
        }

        void PushLayer(Layer* layer) { m_LayerStack.PushLayer(layer); }

    private:
        LayerStack m_LayerStack;
    };
}

// --- Logging ---
#define PRISMA_CORE_INFO(...)  printf("[PRISMA INFO] "); printf(__VA_ARGS__); printf("\n")
#define PRISMA_CORE_TRACE(...) printf("[PRISMA TRACE] "); printf(__VA_ARGS__); printf("\n")
#define PRISMA_CORE_WARN(...)  printf("[PRISMA WARN] "); printf(__VA_ARGS__); printf("\n")
#define PRISMA_CORE_ERROR(...) printf("[PRISMA ERROR] "); printf(__VA_ARGS__); printf("\n")

#define PRISMA_INFO(...)  printf("[APP INFO] "); printf(__VA_ARGS__); printf("\n")
#define PRISMA_TRACE(...) printf("[APP TRACE] "); printf(__VA_ARGS__); printf("\n")
