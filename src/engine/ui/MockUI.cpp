#include "MockUI.h"
#include <cstdarg>
#include <cstdio>
#include <algorithm>

namespace Mock {
namespace UI {

Canvas& Canvas::Get() {
    static Canvas instance;
    return instance;
}

void Canvas::BeginFrame() {
}

void Canvas::EndFrame() {
}

void Canvas::Text(int x, int y, const char* format, ...) {
    if (!m_OutputCallback) return;
    
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    char line[1104];
    snprintf(line, sizeof(line), "[HUD @ (%d,%d)] %s", x, y, buffer);
    m_OutputCallback(line);
}

void Canvas::TextColored(int x, int y, float r, float g, float b, const char* format, ...) {
    if (!m_OutputCallback) return;
    
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    char line[1150];
    snprintf(line, sizeof(line), "[HUD @ (%d,%d)] \033[38;2;%d;%d;%dm%s\033[0m", 
             x, y, 
             static_cast<int>(r * 255),
             static_cast<int>(g * 255),
             static_cast<int>(b * 255),
             buffer);
    m_OutputCallback(line);
}

void Canvas::Rect(int x, int y, int w, int h, float r, float g, float b) {
    if (!m_OutputCallback) return;
    
    char line[256];
    snprintf(line, sizeof(line), 
             "[RECT @ (%d,%d) %dx%d] Color: (%.2f, %.2f, %.2f)",
             x, y, w, h, r, g, b);
    m_OutputCallback(line);
}

void Canvas::ProgressBar(int x, int y, int w, int h, float progress, float r, float g, float b) {
    if (!m_OutputCallback) return;
    
    char line[512];
    snprintf(line, sizeof(line),
             "[PROGRESS @ (%d,%d) %dx%d] %.1f%% | Color: (%.2f, %.2f, %.2f)",
             x, y, w, h, progress * 100, r, g, b);
    m_OutputCallback(line);
}

void Canvas::Separator(int x, int y, int width) {
    if (!m_OutputCallback) return;
    
    char line[128];
    snprintf(line, sizeof(line), "[SEP @ (%d,%d) width=%d] %s",
             x, y, width, "--------------------------------------------------");
    m_OutputCallback(line);
}

Console& Console::Get() {
    static Console instance;
    return instance;
}

void Console::Log(const char* format, ...) {
    if (!m_OutputCallback) return;
    
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    char line[1100];
    snprintf(line, sizeof(line), "[LOG] %s", buffer);
    m_OutputCallback(line);
}

void Console::LogInfo(const char* format, ...) {
    if (!m_OutputCallback) return;
    
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    char line[1100];
    snprintf(line, sizeof(line), "\033[36m[INFO]\033[0m %s", buffer);
    m_OutputCallback(line);
}

void Console::LogWarning(const char* format, ...) {
    if (!m_OutputCallback) return;
    
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    char line[1100];
    snprintf(line, sizeof(line), "\033[33m[WARN]\033[0m %s", buffer);
    m_OutputCallback(line);
}

void Console::LogError(const char* format, ...) {
    if (!m_OutputCallback) return;
    
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    char line[1100];
    snprintf(line, sizeof(line), "\033[31m[ERROR]\033[0m %s", buffer);
    m_OutputCallback(line);
}

}
}
