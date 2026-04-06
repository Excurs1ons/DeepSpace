#include "DeepSpaceApp.h"
#include <EntryPoint.h>

namespace {
inline void PushLayer(Prisma::Layer* layer) {
    Prisma::Application::Get().PushLayer(layer);
}
}

DeepSpaceApp::DeepSpaceApp() {
}

DeepSpaceApp::~DeepSpaceApp() {
}

int DeepSpaceApp::OnInitialize() {
    PRISMA_CORE_INFO("DeepSpace Simulator initialized.");
    
    PushLayer(new SimulationLayer());
    
    return 0;
}

void DeepSpaceApp::OnUpdate(Prisma::Timestep ts) {
    // Basic update logic
    Prisma::Application::OnUpdate(ts); // Important: calls layers
}

void DeepSpaceApp::OnRender() {
    // Render layers logic would go here
}

void DeepSpaceApp::OnEvent(Prisma::Event& e) {
    Prisma::Application::OnEvent(e); // Important: calls layers
}

Prisma::Application* PrismaCreateApplication() {
    return new DeepSpaceApp();
}
