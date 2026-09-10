#pragma once // Évite les inclusions multiples

#include <string>

namespace core {

class Engine {
public:
  explicit Engine(std::string name);
  void start();

private:
  std::string m_name; // Variable membre privée
};

} // namespace core