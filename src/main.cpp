#include "engine/MockEngine.h"
#include "DeepSpaceApp.h"

int main(int argc, char** argv) {
    Mock::Engine& engine = Mock::Engine::Get();
    
    if (engine.Initialize() != 0) {
        std::cerr << "Failed to initialize engine" << std::endl;
        return -1;
    }
    
    DeepSpace::DeepSpaceApp app;
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
