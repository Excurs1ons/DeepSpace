#include "MockEngine.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <sstream>
#include <algorithm>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <fcntl.h>
#include <poll.h>

namespace Mock {

static std::string LogLevelToString(LogLevel level) {
    switch (level) {
        case LogLevel::Trace:   return "TRACE";
        case LogLevel::Debug:   return "DEBUG";
        case LogLevel::Info:    return "INFO ";
        case LogLevel::Warning: return "WARN ";
        case LogLevel::Error:   return "ERROR";
        case LogLevel::Fatal:   return "FATAL";
    }
    return "UNKNOWN";
}

Logger& Logger::Get() {
    static Logger instance;
    return instance;
}

void Logger::Log(LogLevel level, const std::string& category, const std::string& message) {
    if (level < m_MinLevel) return;
    
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::ostringstream oss;
    oss << "[" << LogLevelToString(level) << "] ";
    oss << "[" << category << "] ";
    oss << message;
    
    std::cout << oss.str() << std::endl;
}

void Logger::SetMinLevel(LogLevel level) {
    m_MinLevel = level;
}

InputManager& InputManager::Get() {
    static InputManager instance;
    return instance;
}

bool InputManager::IsKeyPressed(KeyCode key) {
    for (auto k : m_PressedKeys) {
        if (k == key) return true;
    }
    return false;
}

bool InputManager::IsKeyJustPressed(KeyCode key) {
    for (auto k : m_JustPressedKeys) {
        if (k == key) return true;
    }
    return false;
}

void InputManager::SetKeyState(KeyCode key, bool pressed) {
    if (pressed) {
        bool alreadyPressed = false;
        for (auto k : m_PressedKeys) {
            if (k == key) {
                alreadyPressed = true;
                break;
            }
        }
        if (!alreadyPressed) {
            m_PressedKeys.push_back(key);
            m_JustPressedKeys.push_back(key);
        }
    } else {
        m_PressedKeys.erase(
            std::remove(m_PressedKeys.begin(), m_PressedKeys.end(), key),
            m_PressedKeys.end()
        );
    }
}

void InputManager::ClearJustPressed() {
    m_JustPressedKeys.clear();
}

void InputManager::ClearPressedKeys() {
    m_PressedKeys.clear();
}

char InputManager::GetCharInput() {
    char c = m_CharInput;
    m_CharInput = 0;
    return c;
}

void InputManager::SetCharInput(char c) {
    m_CharInput = c;
}

Engine& Engine::Get() {
    static Engine instance;
    return instance;
}

Engine::Engine() {
    m_MainScene = std::make_shared<Scene>("MainScene");
}

Engine::~Engine() = default;

int Engine::Initialize() {
    if (m_Initialized) return 0;
    
    m_Logger.Log(LogLevel::Info, "Engine", "Mock Engine v1.0 initialized");
    m_Initialized = true;
    return 0;
}

int Engine::Run() {
    if (!m_Initialized) {
        Initialize();
    }
    
    m_Running = true;
    m_LastTime = 0.0;
    
    auto startTime = std::chrono::high_resolution_clock::now();
    
    while (m_Running) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(currentTime - startTime).count();
        double dt = elapsed - m_LastTime;
        m_LastTime = elapsed;
        
        ProcessInput();
        
        for (auto& layer : m_Layers) {
            layer->OnUpdate(dt);
        }
        
        for (auto& layer : m_Layers) {
            layer->OnImGuiRender();
        }
        
        m_InputManager.ClearJustPressed();
        m_InputManager.ClearPressedKeys();
        
        if (m_TargetFPS > 0) {
            double targetFrameTime = 1.0 / m_TargetFPS;
            if (dt < targetFrameTime) {
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(targetFrameTime - dt)
                );
            }
        }
    }
    
    return 0;
}

void Engine::Shutdown() {
    m_Running = false;
    
    for (auto& layer : m_Layers) {
        layer->OnDetach();
    }
    m_Layers.clear();
    
    m_Logger.Log(LogLevel::Info, "Engine", "Mock Engine shutdown complete");
}

void Engine::PushLayer(std::unique_ptr<Layer> layer) {
    layer->OnAttach();
    m_Layers.push_back(std::move(layer));
}

void Engine::PopLayer() {
    if (!m_Layers.empty()) {
        m_Layers.back()->OnDetach();
        m_Layers.pop_back();
    }
}

void Engine::Update() {
    auto startTime = std::chrono::high_resolution_clock::now();
    auto currentTime = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(currentTime - startTime).count();
    double dt = elapsed - m_LastTime;
    m_LastTime = elapsed;
    
    for (auto& layer : m_Layers) {
        layer->OnUpdate(dt);
    }
}

void Engine::ProcessInput() {
    struct pollfd pfd;
    pfd.fd = STDIN_FILENO;
    pfd.events = POLLIN;
    
    int ret = poll(&pfd, 1, 0);
    if (ret > 0) {
        char buffer[256];
        ssize_t n = read(STDIN_FILENO, buffer, sizeof(buffer) - 1);
        for (ssize_t i = 0; i < n; i++) {
            char c = buffer[i];
            if (c == '\r' || c == 0) continue;
            m_InputManager.SetCharInput(c);
            
            KeyCode key = KeyCode::Escape;
            if (c >= 'a' && c <= 'z') {
                key = static_cast<KeyCode>(c - 'a' + 65);
            } else if (c >= 'A' && c <= 'Z') {
                key = static_cast<KeyCode>(c);
            } else if (c >= '0' && c <= '9') {
                key = static_cast<KeyCode>(c);
            } else if (c == ' ') {
                key = KeyCode::Space;
            } else if (c == '\n') {
                key = KeyCode::Enter;
            }
            
            m_InputManager.SetKeyState(key, true);
        }
    }
}

std::shared_ptr<GameObject> Scene::FindGameObject(const std::string& name) {
    for (auto& obj : m_GameObjects) {
        if (obj->GetName() == name) {
            return obj;
        }
    }
    return nullptr;
}

void Scene::RemoveGameObject(GameObject* obj) {
    m_GameObjects.erase(
        std::remove_if(m_GameObjects.begin(), m_GameObjects.end(),
            [obj](const std::shared_ptr<GameObject>& o) { return o.get() == obj; }),
        m_GameObjects.end()
    );
}

} // namespace Mock
