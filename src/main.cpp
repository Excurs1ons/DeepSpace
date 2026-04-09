#include "engine/MockEngine.h"
#include "DeepSpaceApp.h"

int main(int argc, char** argv) {
    Mock::Engine& engine = Mock::Engine::Get();
    
    bool headless = false;
    std::string missionFile = "missions/artemis2.conf";
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--headless" || arg == "-h") {
            headless = true;
        } else if (arg == "--mission") {
            headless = true;
            if (i + 1 < argc) {
                std::string missionArg = argv[++i];
                if (missionArg.find('/') == std::string::npos && missionArg.find('.') != std::string::npos) {
                    missionFile = "missions/" + missionArg;
                } else {
                    missionFile = missionArg;
                }
            }
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "Options:\n";
            std::cout << "  --headless, -h  Run in headless mode (automated, no UI)\n";
            std::cout << "  --mission <file> Run headless with mission config file (default: missions/artemis2.conf)\n";
            std::cout << "  --help          Show this help message\n";
            return 0;
        }
    }
    
    if (engine.Initialize() != 0) {
        std::cerr << "Failed to initialize engine" << std::endl;
        return -1;
    }
    
    DeepSpace::DeepSpaceApp app(headless, missionFile);
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
