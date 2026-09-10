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

# --- Type de build par défaut ---------------------------------------------
# Un générateur mono-configuration (Unix Makefiles / Ninja) laisse
# CMAKE_BUILD_TYPE VIDE par défaut => aucun `-O`, code ~10x plus lent (fatal
# pour une simu type LBM). On force "Release" si l'utilisateur n'a rien choisi.
# Pour déboguer :  cmake --preset clang -DCMAKE_BUILD_TYPE=Debug
get_property(_is_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT _is_multi_config AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Type de build" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        Debug Release RelWithDebInfo MinSizeRel)
endif()
message(STATUS "Build type : ${CMAKE_BUILD_TYPE}")

# Optimisations agressives pour NOTRE code en Release (n'affecte pas raylib).
# -march=native : autorisé ici, projet local WSL ; à retirer pour distribuer.
add_compile_options(
    $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CONFIG:Release>>:-O3>
    $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CONFIG:Release>>:-march=native>
)

# Enforce LLVM's libc++ when using Clang (enables modern C++23 headers like <print>)
#
# IMPORTANT : `-stdlib=libc++` ne concerne QUE le C++. On le restreint donc au
# langage CXX via la generator-expression `$<COMPILE_LANGUAGE:CXX>`. Sans cela,
# le flag fuit vers les fichiers .c des dépendances tierces (raylib / GLFW), et
# si CMake choisit GCC comme compilateur C (cas par défaut ici), `cc` rejette
# l'option avec « unrecognized command-line option '-stdlib=libc++' » et le
# build échoue. Le flag de link reste global : l'édition de liens finale de
# l'exécutable est pilotée par clang++ (les .a statiques de raylib n'y voient
# rien), et l'archivage des libs statiques tierces n'utilise pas ce flag.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-stdlib=libc++>)
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

# --- SECTION 4: Third-Party Dependencies (raylib) ---
#
# Stratégie : on récupère et on compile raylib DANS l'arbre de build via le
# module CMake `FetchContent`. Avantages par rapport à un `apt install
# libraylib-dev` :
#   * Reproductible      : la version est figée par `GIT_TAG` ci-dessous, donc
#                          identique sur toutes les machines / la CI.
#   * Sans privilèges    : aucun `sudo`, rien n'est installé hors du projet.
#   * Portable           : fonctionne à l'identique sous WSL, Linux natif, macOS.
#   * À jour             : on choisit la version voulue, pas celle (souvent
#                          ancienne) empaquetée par la distribution.
#
# PRÉREQUIS SYSTÈME (headers de dev nécessaires pour COMPILER raylib sous Linux -
# raylib construit lui-même ses backends X11 / OpenGL depuis les sources) :
#
#   sudo apt install -y libgl1-mesa-dev libx11-dev libxrandr-dev \
#       libxinerama-dev libxcursor-dev libxi-dev libxkbcommon-dev wayland-protocols
#
# Note libc++ : raylib est une bibliothèque *C*. Le flag `-stdlib=libc++` de la
# SECTION 2 est volontairement restreint au langage CXX (voir le commentaire
# détaillé là-bas) pour ne PAS polluer la compilation C de raylib/GLFW. Aucun
# problème d'ABI entre raylib (C) et le reste du projet (C++/libc++).

# `FetchContent` fait partie de la distribution CMake standard (>= 3.11).
include(FetchContent)

# --- Options de configuration de raylib -------------------------------------
# Positionnées AVANT `FetchContent_MakeAvailable` pour que le sous-projet raylib
# les voie. `FORCE` écrase toute valeur en cache d'une configuration précédente.
set(BUILD_EXAMPLES   OFF CACHE BOOL "raylib: ne pas compiler les exemples"   FORCE)
set(BUILD_GAMES      OFF CACHE BOOL "raylib: ne pas compiler les mini-jeux"  FORCE)
# Bibliothèque statique => un seul binaire à distribuer, pas de .so à trouver au
# lancement. Mettre ON pour produire un libraylib.so partagé à la place.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "raylib: édition de liens statique"     FORCE)

# --- Déclaration de la dépendance ------------------------------------------
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        5.5            # <-- VERSION FIGÉE : bumper ici en connaissance de cause
    GIT_SHALLOW    TRUE           # clone superficiel (pas tout l'historique) => plus rapide
    GIT_PROGRESS   TRUE           # affiche la progression du clone lors du configure
)

# Télécharge (au 1er configure, puis mis en cache dans build/_deps/) et ajoute
# raylib comme sous-répertoire : la cible `raylib` devient disponible.
FetchContent_MakeAvailable(raylib)


# --- SECTION 5: Source Discovery & Output Paths ---
# Automatically collect all .cpp files in src/ and subdirectories.
# CONFIGURE_DEPENDS triggers automatic re-configuration when files are added/deleted.
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "src/*.cpp")

# Redirect compiled binary output to the /bin directory at project root
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")


# --- SECTION 6: Target Definition & Configuration ---
# Create the main executable target using all discovered sources
add_executable(${PROJECT_NAME} ${SOURCES})

# Include directories: hand-written headers (include/) and generated headers (build/generated/)
target_include_directories(${PROJECT_NAME} PRIVATE
    include
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

# Link third-party libraries.
# La cible `raylib` (définie en SECTION 4 par FetchContent) est "usage-requiring" :
# elle propage automatiquement ses include directories et ses dépendances système
# (OpenGL, X11/Wayland, pthread, dl, m...) vers ${PROJECT_NAME}. Il suffit donc
# d'un `#include "raylib.h"` dans le code source.
target_link_libraries(${PROJECT_NAME} PRIVATE raylib)

# --- SECTION 7: Output Naming ---
# Format final output binary file name (e.g., bin/mycpp23_v1.0.0_a1b2c3d)
set_target_properties(${PROJECT_NAME} PROPERTIES
    OUTPUT_NAME "${PROJECT_NAME}_v${PROJECT_VERSION}_${GIT_HASH}"
)