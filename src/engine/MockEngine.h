#pragma once
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <chrono>
#include <optional>
#include <cmath>
#include <deque>
#include <algorithm>
#include <iostream>
#include <sstream>

namespace Mock {

// ============================================
// 核心类型定义
// ============================================

struct Vec3d {
    double x, y, z;
    Vec3d(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    Vec3d operator+(const Vec3d& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3d operator-(const Vec3d& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3d operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3d operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3d& operator+=(const Vec3d& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3d& operator-=(const Vec3d& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3d operator-() const { return {-x, -y, -z}; }
    double Length() const { return std::sqrt(x*x + y*y + z*z); }
    double LengthSquared() const { return x*x + y*y + z*z; }
    Vec3d Normalized() const { double l = Length(); return l > 0 ? Vec3d(x/l, y/l, z/l) : Vec3d(); }
    static double Dot(const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    static Vec3d Cross(const Vec3d& a, const Vec3d& b) {
        return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    }
};

struct Vec2 {
    float x, y;
    Vec2(float x = 0, float y = 0) : x(x), y(y) {}
};

// ============================================
// 日志系统
// ============================================

enum class LogLevel { Trace, Debug, Info, Warning, Error, Fatal };

class Logger {
public:
    static Logger& Get();
    
    Logger() = default;
    void Log(LogLevel level, const std::string& category, const std::string& message);
    void SetMinLevel(LogLevel level);
    
    template<typename... Args>
    void Format(LogLevel level, const std::string& category, const char* fmt, Args&&... args) {
        char buffer[4096];
        snprintf(buffer, sizeof(buffer), fmt, std::forward<Args>(args)...);
        Log(level, category, buffer);
    }
    
private:
    LogLevel m_MinLevel = LogLevel::Info;
};

#define MOCK_INFO(...) ::Mock::Logger::Get().Format(::Mock::LogLevel::Info, "Mock", __VA_ARGS__)
#define MOCK_TRACE(...) ::Mock::Logger::Get().Format(::Mock::LogLevel::Trace, "Mock", __VA_ARGS__)
#define MOCK_WARN(...) ::Mock::Logger::Get().Format(::Mock::LogLevel::Warning, "Mock", __VA_ARGS__)
#define MOCK_ERROR(...) ::Mock::Logger::Get().Format(::Mock::LogLevel::Error, "Mock", __VA_ARGS__)

// ============================================
// 输入系统
// ============================================

enum class KeyCode {
    Space = 32, Apostrophe = 39, Comma = 44, Minus = 45, Period = 46, Slash = 47,
    D0 = 48, D1 = 49, D2 = 50, D3 = 51, D4 = 52, D5 = 53, D6 = 54, D7 = 55, D8 = 56, D9 = 57,
    Semicolon = 59, Equal = 61, A = 65, B = 66, C = 67, D = 68, E = 69, F = 70, G = 71,
    H = 72, I = 73, J = 74, K = 75, L = 76, M = 77, N = 78, O = 79, P = 80, Q = 81,
    R = 82, S = 83, T = 84, U = 85, V = 86, W = 87, X = 88, Y = 89, Z = 90,
    LeftBracket = 91, Backslash = 92, RightBracket = 93, GraveAccent = 96,
    World1 = 161, World2 = 162, Escape = 256, Enter = 257, Tab = 258,
    Backspace = 259, Insert = 260, Delete = 261, Right = 262, Left = 263, Down = 264, Up = 265
};

class InputManager {
public:
    static InputManager& Get();
    
    InputManager() = default;
    bool IsKeyPressed(KeyCode key);
    bool IsKeyJustPressed(KeyCode key);
    void SetKeyState(KeyCode key, bool pressed);
    void ClearJustPressed();
    
    char GetCharInput();
    void SetCharInput(char c);
    
private:
    std::vector<KeyCode> m_PressedKeys;
    std::vector<KeyCode> m_JustPressedKeys;
    std::vector<KeyCode> m_LastFrameKeys;
    char m_CharInput = 0;
};

// ============================================
// Layer 系统
// ============================================

class Layer {
public:
    explicit Layer(const std::string& name) : m_Name(name) {}
    virtual ~Layer() = default;
    
    virtual void OnAttach() {}
    virtual void OnDetach() {}
    virtual void OnUpdate(double dt) {}
    virtual void OnImGuiRender() {}
    virtual void OnKeyEvent(KeyCode key, bool pressed) {}
    
    const std::string& GetName() const { return m_Name; }
    
protected:
    std::string m_Name;
};

// ============================================
// 时间系统
// ============================================

class Timestep {
public:
    Timestep(double seconds = 0.0) : m_Seconds(seconds) {}
    double GetSeconds() const { return m_Seconds; }
    double GetMilliseconds() const { return m_Seconds * 1000.0; }
    
private:
    double m_Seconds;
};

// ============================================
// 场景系统 (简化版)
// ============================================

class GameObject {
public:
    GameObject(const std::string& name) : m_Name(name) {}
    
    void SetPosition(const Vec3d& pos) { m_Position = pos; }
    const Vec3d& GetPosition() const { return m_Position; }
    
    void SetOrientation(const Vec3d& dir) { m_Orientation = dir.Normalized(); }
    const Vec3d& GetOrientation() const { return m_Orientation; }
    
    void SetScale(const Vec3d& scale) { m_Scale = scale; }
    const Vec3d& GetScale() const { return m_Scale; }
    
    const std::string& GetName() const { return m_Name; }
    
private:
    std::string m_Name;
    Vec3d m_Position;
    Vec3d m_Orientation{0, 1, 0};
    Vec3d m_Scale{1, 1, 1};
};

class Scene {
public:
    Scene(const std::string& name) : m_Name(name) {}
    
    void AddGameObject(std::shared_ptr<GameObject> obj) { m_GameObjects.push_back(obj); }
    void RemoveGameObject(GameObject* obj);
    
    template<typename T>
    std::shared_ptr<T> CreateGameObject(const std::string& name) {
        auto obj = std::make_shared<T>(name);
        AddGameObject(obj);
        return obj;
    }
    
    std::shared_ptr<GameObject> FindGameObject(const std::string& name);
    
    const std::string& GetName() const { return m_Name; }
    
private:
    std::string m_Name;
    std::vector<std::shared_ptr<GameObject>> m_GameObjects;
};

// ============================================
// 相机系统
// ============================================

enum class CameraMode { Chase, TV, Ground, Free };

class Camera {
public:
    Camera() = default;
    
    void SetMode(CameraMode mode) { m_Mode = mode; }
    CameraMode GetMode() const { return m_Mode; }
    
    void SetPosition(const Vec3d& pos) { m_Position = pos; }
    const Vec3d& GetPosition() const { return m_Position; }
    
    void SetTarget(const Vec3d& target) { m_Target = target; }
    const Vec3d& GetTarget() const { return m_Target; }
    
    void SetFOV(float fov) { m_FOV = fov; }
    float GetFOV() const { return m_FOV; }
    
    void SetDistance(double dist) { m_Distance = dist; }
    double GetDistance() const { return m_Distance; }
    
private:
    CameraMode m_Mode = CameraMode::Chase;
    Vec3d m_Position;
    Vec3d m_Target;
    float m_FOV = 60.0f;
    double m_Distance = 50.0;
};

// ============================================
// 引擎核心
// ============================================

class Engine {
public:
    static Engine& Get();
    
    Engine();
    ~Engine();
    
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;
    
    int Initialize();
    int Run();
    void Shutdown();
    
    // Layer 管理
    void PushLayer(std::unique_ptr<Layer> layer);
    void PopLayer();
    
    // 子系统访问
    InputManager* GetInputManager() { return &m_InputManager; }
    Logger* GetLogger() { return &m_Logger; }
    Scene* GetMainScene() { return m_MainScene.get(); }
    Camera* GetMainCamera() { return &m_MainCamera; }
    
    // 配置
    void SetTargetFPS(int fps) { m_TargetFPS = fps; }
    int GetTargetFPS() const { return m_TargetFPS; }
    
    bool IsRunning() const { return m_Running; }
    
    // 场景管理
    void SetMainScene(std::shared_ptr<Scene> scene) { m_MainScene = scene; }
    
private:
    void Update();
    void ProcessInput();
    
    bool m_Running = false;
    bool m_Initialized = false;
    int m_TargetFPS = 60;
    
    std::vector<std::unique_ptr<Layer>> m_Layers;
    InputManager m_InputManager;
    Logger m_Logger;
    std::shared_ptr<Scene> m_MainScene;
    Camera m_MainCamera;
    
    double m_LastTime = 0.0;
    double m_Accumulator = 0.0;
    double m_FixedDeltaTime = 1.0 / 60.0;
};

// ============================================
// 应用基类
// ============================================

class Application {
public:
    virtual ~Application() = default;
    virtual int OnInitialize() = 0;
    virtual void OnShutdown() {}
};

// 入口点声明
Application* CreateApplication();
int Main(int argc, char** argv);

} // namespace Mock
