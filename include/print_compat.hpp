#pragma once

// Compatibilité <print> (C++23, P2093) : implémenté par libc++ (Clang, build
// natif Linux) mais absent de libstdc++ avant GCC 14 — or le cross-compilateur
// MinGW-w64 disponible sous Ubuntu 24.04 (preset "mingw") est figé à GCC 13.
// On retombe alors sur `std::format` + `std::cout`, disponible depuis GCC 13.
#if __has_include(<print>)
#include <print>
using std::println;
#else
#include <format>
#include <iostream>

template <typename... Args>
void println(std::format_string<Args...> fmt, Args&&... args) {
  std::cout << std::format(fmt, std::forward<Args>(args)...) << '\n';
}

inline void println() { std::cout << '\n'; }
#endif
