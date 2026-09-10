#pragma once

#include "core/ISimulationEngine.hpp"

#include <cstdint>
#include <string>
#include <vector>

// ============================================================================
// LbmEngine — solveur Lattice-Boltzmann 2D, réseau D2Q9, collision BGK.
// ----------------------------------------------------------------------------
// Méthode de Boltzmann sur réseau (LBM) : au lieu de résoudre directement
// Navier-Stokes sur les champs (rho, u, p), on fait évoluer 9 "populations"
// f_i(x, t) — la fraction de particules se déplaçant dans la direction e_i —
// par une alternance de deux étapes locales et explicites :
//
//   1. COLLISION (BGK) : relaxation vers l'équilibre local
//         f_i*  =  f_i  -  omega * (f_i - f_i^eq(rho, u))
//      avec omega = 1/tau. L'équilibre de Maxwell-Boltzmann discrétisé :
//         f_i^eq = w_i rho [ 1 + 3(e_i·u) + 9/2 (e_i·u)^2 - 3/2 |u|^2 ]
//
//   2. ADVECTION (streaming) : chaque population se décale d'une cellule
//         f_i(x + e_i, t+1) = f_i*(x, t)
//
// Grandeurs macroscopiques (récupérées par moments) :
//         rho = Σ f_i        rho u = Σ f_i e_i
//
// Viscosité cinématique du fluide : nu = c_s^2 (tau - 1/2) = (tau - 1/2)/3
// (unités réseau). Stabilité : tau > 1/2 et Mach = |u|/c_s ≲ 0.3.
//
// Réseau D2Q9 (indices utilisés dans tout le code) :
//        6   2   5
//          \ | /
//        3 - 0 - 1
//          / | \
//        7   4   8
//
// Parois : rebond complet ("halfway bounce-back") — une population qui heurte
// une cellule solide repart en sens inverse (condition de non-glissement).
// Entrée (gauche) : vitesse imposée. Sortie (droite) : gradient nul.
// ============================================================================
class LbmEngine final : public ISimulationEngine {
public:
  // Grandeur affichée par render_to_buffer().
  //   Speed     : norme de la vitesse |u|
  //   Vorticity : rotationnel w_z = du_y/dx - du_x/dy
  //   Pressure  : coefficient de pression Cp = (p - p_inf) / (1/2 rho U^2)
  enum class Field { Speed, Vorticity, Pressure };

  // Boîte englobante des cellules solides (en cellules, bornes incluses).
  struct Bounds {
    int x0, y0, x1, y1;
    bool valid;
  };

  LbmEngine() = default;
  ~LbmEngine() override = default;

  // --- Interface ISimulationEngine ---------------------------------------
  void init(int grid_width, int grid_height) override;
  void step(int sub_steps) override;
  void render_to_buffer(std::vector<Color> &pixel_buffer,
                        float max_velocity_scale) override;
  void set_obstacle(int grid_x, int grid_y, bool active) override;
  std::string get_name() const override { return "LBM D2Q9 (BGK)"; }

  // --- Réglages (hors interface) ----------------------------------------
  // Vitesse d'entrée en unités réseau (typiquement 0.02 – 0.1).
  void set_inlet_velocity(double u_lattice) { m_u_in = u_lattice; }
  // Temps de relaxation BGK ; borné à > 0.5 (stabilité). Pilote la viscosité.
  void set_relaxation_time(double tau);
  void set_render_field(Field f) { m_field = f; }
  Field render_field() const { return m_field; }

  // Ré-initialise les populations (équilibre au repos, vitesse d'entrée),
  // sans toucher aux obstacles.
  void reset();

  // Estampille / efface un disque plein d'obstacles (rayon en cellules).
  void stamp_disk(int cx, int cy, int radius, bool active);

  int width() const { return m_w; }
  int height() const { return m_h; }

  // Norme de la vitesse macroscopique en (x, y) — utile pour tests / mesures.
  double speed_at(int x, int y) const;

  // Nombre de Reynolds pour une échelle caractéristique donnée (en cellules).
  double reynolds(double length_scale) const;

  // --- Efforts aérodynamiques sur les obstacles -------------------------
  // Calculés par la méthode d'échange de quantité de mouvement (momentum
  // exchange) sur les liens de rebond, moyennés sur les sous-pas puis lissés
  // (EMA). Unités réseau. Repère : x = aval (traînée), y = vers le HAUT écran.
  double drag() const { return m_fx_ema; }  // F_x  (traînée / Cx)
  double lift() const { return -m_fy_ema; } // -F_y (portance vers le haut)
  double drag_coefficient() const;          // Cx = F_x / (1/2 rho U^2 c)
  double lift_coefficient() const;          // Cz = portance / (1/2 rho U^2 c)

  Bounds solid_bounds() const { return m_bounds; }

private:
  int idx(int x, int y) const { return y * m_w + x; }
  void equilibrium_at(int n, double rho, double ux, double uy);
  void apply_boundaries();
  void collide_and_stream(double &fx, double &fy);
  void compute_macros();
  void update_solid_bounds();

  int m_w = 0;
  int m_h = 0;

  double m_tau = 0.6;         // temps de relaxation BGK
  double m_omega = 1.0 / 0.6; // = 1 / tau
  double m_u_in = 0.08;       // vitesse d'entrée (unités réseau)

  Field m_field = Field::Vorticity;

  // Populations : disposition SoA — m_f[dir * N + cell], N = m_w * m_h.
  std::vector<double> m_f;     // état courant
  std::vector<double> m_f_tmp; // cible du streaming (ping-pong)

  // Grandeurs macroscopiques, recalculées en fin de step() pour le rendu.
  std::vector<double> m_rho;
  std::vector<double> m_ux;
  std::vector<double> m_uy;

  std::vector<std::uint8_t> m_solid; // 1 = paroi solide

  // Efforts : moyenne sur le dernier step() (m_fx/m_fy) et version lissée
  // (EMA).
  double m_fx = 0.0;
  double m_fy = 0.0;
  double m_fx_ema = 0.0;
  double m_fy_ema = 0.0;
  Bounds m_bounds{0, 0, 0, 0, false};
};
