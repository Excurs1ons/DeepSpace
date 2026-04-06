#pragma once
#include "PrismaEngine.h"

extern Prisma::Application* PrismaCreateApplication();

int main(int argc, char** argv) {
    PRISMA_CORE_INFO("Prisma Engine Mock Initializing...");
    
    auto app = PrismaCreateApplication();
    if (app->OnInitialize() == 0) {
        PRISMA_CORE_INFO("Entering Simulation Loop (500 seconds of flight)...");
        
        Prisma::Timestep ts(0.016f);
        // Run for 31250 frames = 500 seconds
        for (int i = 0; i < 31250; ++i) {
            app->OnUpdate(ts);
            app->OnRender();
        }
    }
    
    delete app;
    PRISMA_CORE_INFO("Simulation Finished.");
    return 0;
}
