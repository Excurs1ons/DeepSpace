#include "DeepSpaceApp.h"
#include <PrismaEngine/EntryPoint.h>

Prisma::Application* PrismaCreateApplication() {
    return new DeepSpaceApp();
}

int main(int argc, char** argv) {
    Prisma::EngineSpecification spec;
    spec.Name = "DeepSpace";
    
    Prisma::Engine engine(spec);
    
    if (engine.Initialize() != 0) {
        return -1;
    }

    auto* app = PrismaCreateApplication();
    int result = engine.Run(std::unique_ptr<Prisma::Application>(app));

    engine.Shutdown();

    return result;
}
