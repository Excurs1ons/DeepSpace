#pragma once
#include <string>
#include <vector>
#include <functional>

namespace Mock {
namespace UI {

class Canvas {
public:
    static Canvas& Get();
    
    void BeginFrame();
    void EndFrame();
    
    void Text(int x, int y, const char* format, ...);
    void TextColored(int x, int y, float r, float g, float b, const char* format, ...);
    void Rect(int x, int y, int w, int h, float r, float g, float b);
    void ProgressBar(int x, int y, int w, int h, float progress, float r, float g, float b);
    void Separator(int x, int y, int width);
    
    void SetOutputCallback(std::function<void(const std::string&)> callback) {
        m_OutputCallback = callback;
    }
    
private:
    Canvas() = default;
    std::function<void(const std::string&)> m_OutputCallback;
};

class Console {
public:
    static Console& Get();
    
    void Log(const char* format, ...);
    void LogInfo(const char* format, ...);
    void LogWarning(const char* format, ...);
    void LogError(const char* format, ...);
    
    void SetOutputCallback(std::function<void(const std::string&)> callback) {
        m_OutputCallback = callback;
    }
    
private:
    Console() = default;
    std::function<void(const std::string&)> m_OutputCallback;
};

}
}

#define UI_TEXT(x, y, fmt, ...) Mock::UI::Canvas::Get().Text(x, y, fmt, ##__VA_ARGS__)
#define UI_TEXT_COLORED(x, y, r, g, b, fmt, ...) Mock::UI::Canvas::Get().TextColored(x, y, r, g, b, fmt, ##__VA_ARGS__)
#define UI_RECT(x, y, w, h, r, g, b) Mock::UI::Canvas::Get().Rect(x, y, w, h, r, g, b)
#define UI_PROGRESS(x, y, w, h, prog, r, g, b) Mock::UI::Canvas::Get().ProgressBar(x, y, w, h, prog, r, g, b)
#define UI_SEP(x, y, w) Mock::UI::Canvas::Get().Separator(x, y, w)

#define LOG_INFO(fmt, ...) Mock::UI::Console::Get().LogInfo(fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) Mock::UI::Console::Get().LogWarning(fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) Mock::UI::Console::Get().LogError(fmt, ##__VA_ARGS__)
