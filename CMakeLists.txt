cmake_minimum_required(VERSION 3.31)
project(DeepSpace VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(USE_MOCK_ENGINE "Use Mock Engine (no dependencies required)" ON)

if(USE_MOCK_ENGINE)
    message(STATUS "Building with Mock Engine (no external dependencies)")
    
    set(MOCK_SOURCES
        src/engine/MockEngine.cpp
        src/engine/ui/MockUI.cpp
    )
    
    file(GLOB_RECURSE CORE_SOURCES "src/core/*.cpp" "src/core/*.h")
    file(GLOB_RECURSE PHYSICS_SOURCES "src/physics/*.cpp" "src/physics/*.h")
    file(GLOB_RECURSE VESSEL_SOURCES "src/vessel/*.cpp" "src/vessel/*.h")
    file(GLOB_RECURSE ENVIRONMENT_SOURCES "src/environment/*.cpp" "src/environment/*.h")
    
    set(ALL_SOURCES
        ${MOCK_SOURCES}
        ${CORE_SOURCES}
        ${PHYSICS_SOURCES}
        ${VESSEL_SOURCES}
        ${ENVIRONMENT_SOURCES}
        src/main.cpp
    )
    
    add_executable(DeepSpace ${ALL_SOURCES})
    target_include_directories(DeepSpace PRIVATE src)
    target_compile_definitions(DeepSpace PRIVATE USE_MOCK_ENGINE)
    message(STATUS "Mock Engine mode: Text-based simulation enabled")

else()
    message(FATAL_ERROR "PrismaEngine SDK mode not yet supported. Use -DUSE_MOCK_ENGINE=ON")
endif()

set_target_properties(DeepSpace PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
