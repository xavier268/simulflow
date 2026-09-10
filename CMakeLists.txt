# ==============================================================================
# Modern C++23 CMake Boilerplate
# Designed for Clang/libc++, WSL/Ubuntu, and automated metadata injection.
# ==============================================================================

# --- SECTION 1: Minimum Requirements & Dynamic Project Initialization ---
cmake_minimum_required(VERSION 3.20)

# Automatically set the project name to the parent directory name
get_filename_component(PROJECT_NAME_AUTO ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME_AUTO} VERSION 1.0.1 LANGUAGES CXX)


# --- SECTION 2: C++ Toolchain & Compiler Options ---
# Force C++23 standard enforcement across all targets
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile_commands.json in build/ for clangd & VS Code IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enforce LLVM's libc++ when using Clang (enables modern C++23 headers like <print>)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-stdlib=libc++)
    add_link_options(-stdlib=libc++)
endif()


# --- SECTION 3: Metadata Extraction & Code Generation ---

# 1. Get Compiler Info (ex: "Clang 18.1.3")
set(COMPILER_INFO "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

# 2. Get Git Hash
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT GIT_HASH)
    set(GIT_HASH "unknown")
endif()

# 3. Get Build Time
string(TIMESTAMP BUILD_TIME "%Y-%m-%dT%H:%M:%SZ" UTC)

# 4. Generate version.hpp
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/version.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/version.hpp
    @ONLY
)

# --- SECTION 4: Source Discovery & Output Paths ---
# Automatically collect all .cpp files in src/ and subdirectories.
# CONFIGURE_DEPENDS triggers automatic re-configuration when files are added/deleted.
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "src/*.cpp")

# Redirect compiled binary output to the /bin directory at project root
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")


# --- SECTION 5: Target Definition & Configuration ---
# Create the main executable target using all discovered sources
add_executable(${PROJECT_NAME} ${SOURCES})

# Include directories: hand-written headers (include/) and generated headers (build/generated/)
target_include_directories(${PROJECT_NAME} PRIVATE
    include
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

# Format final output binary file name (e.g., bin/mycpp23_v1.0.0_a1b2c3d)
set_target_properties(${PROJECT_NAME} PROPERTIES
    OUTPUT_NAME "${PROJECT_NAME}_v${PROJECT_VERSION}_${GIT_HASH}"
)