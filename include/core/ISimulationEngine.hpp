#pragma once
#include "raylib.h"
#include <string>
#include <vector>

class ISimulationEngine {
public:
  virtual ~ISimulationEngine() = default;

  // Réinitialise le domaine et les conditions aux limites
  virtual void init(int grid_width, int grid_height) = 0;

  // Effectue N pas de calcul physique (Sub-stepping)
  virtual void step(int sub_steps) = 0;

  // Rempli le buffer de pixels RGBA pour la texture Raylib
  virtual void render_to_buffer(std::vector<Color> &pixel_buffer,
                                float max_velocity_scale) = 0;

  // Interaction utilisateur : ajoute ou retire un obstacle à la position (x, y)
  // de la grille
  virtual void set_obstacle(int grid_x, int grid_y, bool active) = 0;

  // Nom du moteur pour l'IHM
  virtual std::string get_name() const = 0;
};