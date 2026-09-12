# Toolchain de cross-compilation Linux -> Windows (x86_64), via MinGW-w64.
#
# Prérequis :  sudo apt install -y mingw-w64
#
# Usage :  cmake --preset mingw && cmake --build --preset mingw
#
# Produit un exécutable Windows autonome (bibliothèques C/C++ standard et
# raylib liées statiquement, voir SECTION 2 de CMakeLists.txt) : aucune DLL
# à distribuer à côté du .exe.

set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

set(CMAKE_C_COMPILER   x86_64-w64-mingw32-gcc)
set(CMAKE_CXX_COMPILER x86_64-w64-mingw32-g++)
set(CMAKE_RC_COMPILER  x86_64-w64-mingw32-windres)

# Où chercher d'éventuelles libs/headers déjà installés pour la cible mingw
# (ne concerne pas raylib, récupéré et compilé depuis les sources par
# FetchContent, donc toujours cohérent avec le compilateur choisi ici).
set(CMAKE_FIND_ROOT_PATH /usr/x86_64-w64-mingw32)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
