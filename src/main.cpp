#include "engine/MockEngine.h"
#include "DeepSpaceApp.h"

int main(int argc, char** argv) {
    Mock::Engine& engine = Mock::Engine::Get();
    
    bool headless = false;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--headless" || arg == "-h") {
            headless = true;
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "Options:\n";
            std::cout << "  --headless, -h  Run in headless mode (automated, no UI)\n";
            std::cout << "  --help          Show this help message\n";
            return 0;
        }
    }
    
    if (engine.Initialize() != 0) {
        std::cerr << "Failed to initialize engine" << std::endl;
        return -1;
    }
    
    DeepSpace::DeepSpaceApp app(headless);
    const int result = app.OnInitialize();
    
    if (result != 0) {
        std::cerr << "Failed to initialize application" << std::endl;
        return result;
    }
    
    app.Run(engine);
    
    app.OnShutdown();
    engine.Shutdown();
    
    return 0;
}
