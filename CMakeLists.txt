cmake_minimum_required(VERSION 3.31)
project(DeepSpace VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PrismaEngine SDK URL
set(PRISMA_SDK_VERSION "1.0.4")
set(PRISMA_SDK_URL "https://github.com/Excurs1ons/PrismaEngine/releases/download/v${PRISMA_SDK_VERSION}-sdk/PrismaEngine-SDK-${PRISMA_SDK_VERSION}-linux-arm64.tar.gz")

# Download and extract SDK if not present
set(PRISMA_SDK_DIR "${CMAKE_BINARY_DIR}/PrismaEngine-SDK")
if(NOT EXISTS "${PRISMA_SDK_DIR}/cmake/PrismaEngineConfig.cmake")
    message(STATUS "Downloading PrismaEngine SDK v${PRISMA_SDK_VERSION}...")
    file(DOWNLOAD "${PRISMA_SDK_URL}" "${CMAKE_BINARY_DIR}/PrismaEngine-SDK.tar.gz" SHOW_PROGRESS)
    execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf "${CMAKE_BINARY_DIR}/PrismaEngine-SDK.tar.gz"
                    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")
    file(REMOVE "${CMAKE_BINARY_DIR}/PrismaEngine-SDK.tar.gz")
    file(RENAME "${CMAKE_BINARY_DIR}/PrismaEngine-SDK-${PRISMA_SDK_VERSION}-linux-arm64" "${PRISMA_SDK_DIR}")
endif()

# Add SDK to CMake prefix path for find_package
list(APPEND CMAKE_PREFIX_PATH "${PRISMA_SDK_DIR}")

# Find PrismaEngine package
find_package(PrismaEngine REQUIRED)

file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.h")

add_executable(DeepSpace ${SOURCES})

target_include_directories(DeepSpace PRIVATE
    ${PRISMA_SDK_DIR}/include
    ${PRISMA_SDK_DIR}/third_party
    src
)

target_link_libraries(DeepSpace PRIVATE PrismaEngine::Engine)

# Add SDK lib directory for linking at runtime
target_link_directories(DeepSpace PRIVATE "${PRISMA_SDK_DIR}/lib/linux")

set_target_properties(DeepSpace PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
