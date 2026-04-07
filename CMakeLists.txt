cmake_minimum_required(VERSION 3.31)
project(DeepSpace VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PRISMA_SDK_VERSION "1.0.4")
set(PRISMA_SDK_DIR "" CACHE PATH "Path to a local PrismaEngine SDK. Leave empty to auto-download when supported.")

if(NOT PRISMA_SDK_DIR)
    set(_prisma_triplet "")

    if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
        set(_prisma_triplet "linux-arm64")
    endif()

    if(NOT _prisma_triplet)
        if(WIN32)
            message(FATAL_ERROR
                "PrismaEngine Windows x64 SDK is not published yet. "
                "Please build on Linux ARM64 for now, or set PRISMA_SDK_DIR to a manually prepared SDK path.")
        endif()

        message(FATAL_ERROR
            "No auto-download PrismaEngine SDK is available for this platform: "
            "${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}. "
            "Set PRISMA_SDK_DIR to a local SDK path.")
    endif()

    set(PRISMA_SDK_URL
        "https://github.com/Excurs1ons/PrismaEngine/releases/download/v${PRISMA_SDK_VERSION}-sdk/PrismaEngine-SDK-${PRISMA_SDK_VERSION}-${_prisma_triplet}.tar.gz")

    set(PRISMA_SDK_DIR "${CMAKE_BINARY_DIR}/PrismaEngine-SDK")

    if(NOT EXISTS "${PRISMA_SDK_DIR}/cmake/PrismaEngineConfig.cmake")
        message(STATUS "Downloading PrismaEngine SDK v${PRISMA_SDK_VERSION} (${_prisma_triplet})...")
        file(DOWNLOAD "${PRISMA_SDK_URL}" "${CMAKE_BINARY_DIR}/PrismaEngine-SDK.tar.gz" SHOW_PROGRESS)
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E tar xzf "${CMAKE_BINARY_DIR}/PrismaEngine-SDK.tar.gz"
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")
        file(REMOVE "${CMAKE_BINARY_DIR}/PrismaEngine-SDK.tar.gz")
        file(RENAME "${CMAKE_BINARY_DIR}/PrismaEngine-SDK-${PRISMA_SDK_VERSION}-${_prisma_triplet}" "${PRISMA_SDK_DIR}")
    endif()
endif()

list(APPEND CMAKE_PREFIX_PATH "${PRISMA_SDK_DIR}")
find_package(PrismaEngine REQUIRED)

file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.h")
add_executable(DeepSpace ${SOURCES})

target_include_directories(DeepSpace PRIVATE
    "${PRISMA_SDK_DIR}/include"
    "${PRISMA_SDK_DIR}/third_party"
    src)

target_link_libraries(DeepSpace PRIVATE PrismaEngine::Engine)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_directories(DeepSpace PRIVATE "${PRISMA_SDK_DIR}/lib/linux")
endif()

set_target_properties(DeepSpace PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
