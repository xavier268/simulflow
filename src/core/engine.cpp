#include "core/engine.hpp"
#include <iostream>

namespace core {

Engine::Engine(std::string name) : m_name(std::move(name)) {}

void Engine::start() {
  std::cout << "Moteur " << m_name << " démarré." << std::endl;
}

} // namespace core