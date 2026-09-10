#pragma once

#include "core/ISimulationEngine.hpp"

#include <cstdint>
#include <string>
#include <vector>

// ============================================================================
// FluidEngine
// ----------------------------------------------------------------------------
// Implémentation de référence (SQUELETTE) de ISimulationEngine.
//
// À ce stade ce n'est PAS encore un vrai solveur de Navier-Stokes : c'est un
// champ scalaire de "densité" (fumée) sur lequel on applique une diffusion
// simple + une dissipation, avec des obstacles qui remettent la densité à zéro.
// L'objectif est de valider toute la chaîne : moteur -> buffer de pixels ->
// texture Raylib -> affichage -> interaction souris.
//
// Étapes suivantes prévues :
//   * ajouter un champ de vitesse (u, v) et l'advection de la densité,
//   * projection (incompressibilité) façon Jos Stam / "Stable Fluids",
//   * conditions aux limites sur les obstacles.
// ============================================================================
class FluidEngine final : public ISimulationEngine {
public:
  FluidEngine() = default;
  ~FluidEngine() override = default;

  // --- Interface ISimulationEngine -----------------------------------------
  void init(int grid_width, int grid_height) override;
  void step(int sub_steps) override;
  void render_to_buffer(std::vector<Color> &pixel_buffer,
                        float max_velocity_scale) override;
  void set_obstacle(int grid_x, int grid_y, bool active) override;
  std::string get_name() const override { return "FluidEngine (skeleton)"; }

  // --- API additionnelle (hors interface) ---------------------------------
  // Injecte de la densité à la position grille (x, y) — utilisé par la souris.
  void add_density(int grid_x, int grid_y, float amount);

  int width() const { return m_width; }
  int height() const { return m_height; }

private:
  // Convertit des coordonnées grille (x, y) en index linéaire du buffer.
  // Renvoie -1 si (x, y) est hors domaine.
  int index(int x, int y) const;

  int m_width = 0;  // nombre de cellules en X
  int m_height = 0; // nombre de cellules en Y

  float m_diffusion = 0.10f;   // coefficient de diffusion par sous-pas
  float m_dissipation = 0.99f; // densité *= dissipation à chaque sous-pas

  std::vector<float> m_density;      // champ courant, taille width*height
  std::vector<float> m_density_prev; // tampon de calcul (ping-pong)
  std::vector<std::uint8_t> m_solid; // 1 = obstacle, 0 = fluide
};
