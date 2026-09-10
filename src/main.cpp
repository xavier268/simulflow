#include "core/engine.hpp"
#include "version.hpp"
#include <print>

int main() {

  std::print("Hello, C++23 template project!\n");

  std::println("Application:  v{}", version::VERSION);
  std::println("Git Commit:   {}", version::GIT_HASH);
  std::println("Build Time:   {}", version::BUILD_TIME);
  std::println("Compiler:     {}", version::COMPILER);

  core::Engine engine("V8");
  engine.start();
  return 0;
}