#include "core/FluidEngine.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>

// ----------------------------------------------------------------------------
// Helpers internes
// ----------------------------------------------------------------------------
int FluidEngine::index(int x, int y) const {
  if (x < 0 || y < 0 || x >= m_width || y >= m_height)
    return -1;
  return y * m_width + x;
}

// ----------------------------------------------------------------------------
// init : (ré)alloue le domaine et remet tout à zéro.
// ----------------------------------------------------------------------------
void FluidEngine::init(int grid_width, int grid_height) {
  m_width = std::max(grid_width, 1);
  m_height = std::max(grid_height, 1);

  const std::size_t cells = static_cast<std::size_t>(m_width) * m_height;
  m_density.assign(cells, 0.0f);
  m_density_prev.assign(cells, 0.0f);
  m_solid.assign(cells, 0);
}

// ----------------------------------------------------------------------------
// add_density : injection ponctuelle (clampée dans [0, 1]).
// ----------------------------------------------------------------------------
void FluidEngine::add_density(int grid_x, int grid_y, float amount) {
  const int i = index(grid_x, grid_y);
  if (i < 0 || m_solid[i])
    return;
  m_density[i] = std::clamp(m_density[i] + amount, 0.0f, 1.0f);
}

// ----------------------------------------------------------------------------
// set_obstacle : marque / démarque une cellule solide.
// ----------------------------------------------------------------------------
void FluidEngine::set_obstacle(int grid_x, int grid_y, bool active) {
  const int i = index(grid_x, grid_y);
  if (i < 0)
    return;
  m_solid[i] = active ? 1 : 0;
  if (active)
    m_density[i] = 0.0f;
}

// ----------------------------------------------------------------------------
// step : `sub_steps` itérations de diffusion + dissipation.
//
// Diffusion explicite type "moyenne des 4 voisins" :
//   d'(x) = d(x) + k * (somme_voisins - 4*d(x))
// Les cellules solides ne participent pas aux échanges (Neumann grossier).
// Schéma volontairement simple : suffisant pour un squelette, à remplacer par
// une résolution implicite (Gauss-Seidel) quand on ajoutera la vitesse.
// ----------------------------------------------------------------------------
void FluidEngine::step(int sub_steps) {
  if (m_density.empty())
    return;

  for (int s = 0; s < std::max(sub_steps, 1); ++s) {
    m_density_prev = m_density;

    for (int y = 0; y < m_height; ++y) {
      for (int x = 0; x < m_width; ++x) {
        const int i = index(x, y);
        if (m_solid[i]) {
          m_density[i] = 0.0f;
          continue;
        }

        const float center = m_density_prev[i];
        float acc = 0.0f;
        int n = 0;

        // Voisins von Neumann ; un voisin hors domaine ou solide est ignoré.
        for (const auto &[dx, dy] : {std::pair{1, 0}, std::pair{-1, 0},
                                     std::pair{0, 1}, std::pair{0, -1}}) {
          const int j = index(x + dx, y + dy);
          if (j < 0 || m_solid[j])
            continue;
          acc += m_density_prev[j];
          ++n;
        }

        const float diffused = center + m_diffusion * (acc - n * center);
        m_density[i] = std::clamp(diffused * m_dissipation, 0.0f, 1.0f);
      }
    }
  }
}

// ----------------------------------------------------------------------------
// render_to_buffer : mappe la densité vers un buffer RGBA prêt pour Raylib.
//
// `max_velocity_scale` : facteur d'intensité global. Nommé ainsi car il servira
// à normaliser la magnitude de vitesse une fois le champ (u, v) implémenté ;
// pour l'instant il module simplement la luminosité de la fumée.
// ----------------------------------------------------------------------------
void FluidEngine::render_to_buffer(std::vector<Color> &pixel_buffer,
                                   float max_velocity_scale) {
  const std::size_t cells = static_cast<std::size_t>(m_width) * m_height;
  if (pixel_buffer.size() != cells)
    pixel_buffer.assign(cells, Color{0, 0, 0, 255});

  const float scale = (max_velocity_scale > 0.0f) ? max_velocity_scale : 1.0f;

  for (std::size_t i = 0; i < cells; ++i) {
    if (m_solid[i]) {
      pixel_buffer[i] = Color{70, 70, 90, 255}; // obstacle : gris bleuté
      continue;
    }

    const float d = std::clamp(m_density[i] * scale, 0.0f, 1.0f);
    const auto v = static_cast<unsigned char>(d * 255.0f);
    // Dégradé sombre -> cyan clair.
    pixel_buffer[i] = Color{static_cast<unsigned char>(v / 3), v,
                            static_cast<unsigned char>(180 + v / 4), 255};
  }
}
