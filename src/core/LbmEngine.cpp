#include "core/LbmEngine.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

// ============================================================================
// Constantes du réseau D2Q9 et helpers locaux (non exposés).
// ============================================================================
namespace {

constexpr int kQ = 9;

// Vecteurs vitesse e_i (voir le schéma dans l'en-tête).
constexpr int kCx[kQ] = {0, 1, 0, -1, 0, 1, -1, -1, 1};
constexpr int kCy[kQ] = {0, 0, 1, 0, -1, 1, 1, -1, -1};

// Poids du modèle : 4/9 au centre, 1/9 sur les axes, 1/36 sur les diagonales.
constexpr double kW[kQ] = {4.0 / 9.0,  1.0 / 9.0,  1.0 / 9.0,
                           1.0 / 9.0,  1.0 / 9.0,  1.0 / 36.0,
                           1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0};

// Direction opposée e_{opp[i]} = -e_i, utilisée par le rebond (bounce-back).
constexpr int kOpp[kQ] = {0, 3, 4, 1, 2, 7, 8, 5, 6};

// 1 / c_s^2  avec  c_s^2 = 1/3  (vitesse du son du réseau).
constexpr double kInvCs2 = 3.0;

// Distribution d'équilibre dans la direction i.
inline double feq_dir(int i, double rho, double ux, double uy) {
  const double eu = kCx[i] * ux + kCy[i] * uy;
  const double usq = ux * ux + uy * uy;
  return kW[i] * rho *
         (1.0 + kInvCs2 * eu + 0.5 * kInvCs2 * kInvCs2 * eu * eu -
          0.5 * kInvCs2 * usq);
}

// --- Colormaps -----------------------------------------------------------
struct Rgb {
  unsigned char r, g, b;
};

inline Rgb lerp(Rgb a, Rgb b, double t) {
  t = std::clamp(t, 0.0, 1.0);
  return {static_cast<unsigned char>(a.r + (b.r - a.r) * t),
          static_cast<unsigned char>(a.g + (b.g - a.g) * t),
          static_cast<unsigned char>(a.b + (b.b - a.b) * t)};
}

// Séquentielle "sombre -> feu -> clair" pour la norme de la vitesse (t in
// [0,1]).
inline Rgb colormap_speed(double t) {
  static constexpr Rgb stops[5] = {{8, 8, 20},
                                   {40, 20, 120},
                                   {180, 40, 90},
                                   {250, 170, 40},
                                   {255, 255, 220}};
  const double x = std::clamp(t, 0.0, 1.0) * 4.0;
  const int k = std::min(3, static_cast<int>(x));
  return lerp(stops[k], stops[k + 1], x - k);
}

// Divergente bleu <- sombre -> rouge (t in [-1, 1]).
// Sert à la vorticité (sens de rotation) ET à la pression
// (dépression/surpression).
inline Rgb colormap_diverging(double t) {
  t = std::clamp(t, -1.0, 1.0);
  const Rgb mid{10, 12, 18};
  return t < 0.0 ? lerp(mid, Rgb{60, 120, 255}, -t) // bleu : négatif
                 : lerp(mid, Rgb{255, 80, 60}, t);  // rouge : positif
}

} // namespace

// ============================================================================
// Cycle de vie
// ============================================================================
void LbmEngine::set_relaxation_time(double tau) {
  m_tau = std::max(tau, 0.5001); // tau > 1/2 impératif pour la stabilité
  m_omega = 1.0 / m_tau;
}

double LbmEngine::reynolds(double length_scale) const {
  const double nu = (m_tau - 0.5) / kInvCs2; // nu = c_s^2 (tau - 1/2)
  return nu > 0.0 ? m_u_in * length_scale / nu : 0.0;
}

// Coefficients : effort / (1/2 rho_inf U_inf^2 c), c = corde = étendue en x du
// solide. rho_inf = 1 en unités réseau.
double LbmEngine::drag_coefficient() const {
  if (!m_bounds.valid)
    return 0.0;
  const double c = m_bounds.x1 - m_bounds.x0 + 1;
  const double q = 0.5 * m_u_in * m_u_in * c;
  return q > 0.0 ? m_fx_ema / q : 0.0;
}

double LbmEngine::lift_coefficient() const {
  if (!m_bounds.valid)
    return 0.0;
  const double c = m_bounds.x1 - m_bounds.x0 + 1;
  const double q = 0.5 * m_u_in * m_u_in * c;
  return q > 0.0 ? -m_fy_ema / q : 0.0; // portance = composante "vers le haut"
}

void LbmEngine::update_solid_bounds() {
  Bounds b{m_w, m_h, -1, -1, false};
  for (int y = 0; y < m_h; ++y)
    for (int x = 0; x < m_w; ++x)
      if (m_solid[idx(x, y)]) {
        b.x0 = std::min(b.x0, x);
        b.y0 = std::min(b.y0, y);
        b.x1 = std::max(b.x1, x);
        b.y1 = std::max(b.y1, y);
        b.valid = true;
      }
  m_bounds = b;
}

double LbmEngine::speed_at(int x, int y) const {
  if (x < 0 || y < 0 || x >= m_w || y >= m_h)
    return 0.0;
  const int n = idx(x, y);
  return std::sqrt(m_ux[n] * m_ux[n] + m_uy[n] * m_uy[n]);
}

void LbmEngine::equilibrium_at(int n, double rho, double ux, double uy) {
  const int N = m_w * m_h;
  for (int i = 0; i < kQ; ++i)
    m_f[i * N + n] = feq_dir(i, rho, ux, uy);
}

void LbmEngine::init(int grid_width, int grid_height) {
  m_w = std::max(grid_width, 8);
  m_h = std::max(grid_height, 8);
  const std::size_t N = static_cast<std::size_t>(m_w) * m_h;

  m_f.assign(N * kQ, 0.0);
  m_f_tmp.assign(N * kQ, 0.0);
  m_rho.assign(N, 1.0);
  m_ux.assign(N, 0.0);
  m_uy.assign(N, 0.0);
  m_solid.assign(N, 0);

  reset();

  // Obstacle par défaut : un disque légèrement décalé vers le bas pour briser
  // la symétrie et amorcer l'allée tourbillonnaire de von Kármán.
  stamp_disk(m_w / 5, m_h / 2 + m_h / 40 + 1, std::max(3, m_h / 10), true);
}

void LbmEngine::reset() {
  const int N = m_w * m_h;
  for (int n = 0; n < N; ++n) {
    const double u = m_solid[n] ? 0.0 : m_u_in;
    m_rho[n] = 1.0;
    m_ux[n] = u;
    m_uy[n] = 0.0;
    equilibrium_at(n, 1.0, u, 0.0);
  }
}

// ============================================================================
// Obstacles
// ============================================================================
void LbmEngine::set_obstacle(int grid_x, int grid_y, bool active) {
  if (grid_x < 0 || grid_y < 0 || grid_x >= m_w || grid_y >= m_h)
    return;

  const int n = idx(grid_x, grid_y);
  const bool was_solid = m_solid[n] != 0;
  m_solid[n] = active ? 1 : 0;

  // Cellule qui redevient fluide : ses populations sont périmées, on les
  // réamorce à l'équilibre au repos d'entrée.
  if (was_solid && !active) {
    m_rho[n] = 1.0;
    m_ux[n] = m_u_in;
    m_uy[n] = 0.0;
    equilibrium_at(n, 1.0, m_u_in, 0.0);
  }
}

void LbmEngine::stamp_disk(int cx, int cy, int radius, bool active) {
  const int r2 = radius * radius;
  for (int y = cy - radius; y <= cy + radius; ++y) {
    for (int x = cx - radius; x <= cx + radius; ++x) {
      const int dx = x - cx;
      const int dy = y - cy;
      if (dx * dx + dy * dy <= r2)
        set_obstacle(x, y, active);
    }
  }
}

// ============================================================================
// Intégration temporelle
// ============================================================================
void LbmEngine::apply_boundaries() {
  const int N = m_w * m_h;

  // Entrée (x = 0) : vitesse imposée via la distribution d'équilibre.
  for (int y = 0; y < m_h; ++y) {
    const int n = idx(0, y);
    if (m_solid[n])
      continue;
    for (int i = 0; i < kQ; ++i)
      m_f[i * N + n] = feq_dir(i, 1.0, m_u_in, 0.0);
  }

  // Sortie (x = W-1) : gradient nul — recopie de l'avant-dernière colonne.
  for (int y = 0; y < m_h; ++y) {
    const int n = idx(m_w - 1, y);
    const int nm = idx(m_w - 2, y);
    if (m_solid[n])
      continue;
    for (int i = 0; i < kQ; ++i)
      m_f[i * N + n] = m_f[i * N + nm];
  }
}

void LbmEngine::collide_and_stream(double &fx, double &fy) {
  const int N = m_w * m_h;
  fx = 0.0;
  fy = 0.0;

  for (int y = 0; y < m_h; ++y) {
    for (int x = 0; x < m_w; ++x) {
      const int n = idx(x, y);
      if (m_solid[n])
        continue; // les parois ne collisionnent pas

      // --- moments : densité et quantité de mouvement locales ---
      double rho = 0.0;
      double mx = 0.0;
      double my = 0.0;
      for (int i = 0; i < kQ; ++i) {
        const double fi = m_f[i * N + n];
        rho += fi;
        mx += kCx[i] * fi;
        my += kCy[i] * fi;
      }
      const double ux = mx / rho;
      const double uy = my / rho;

      // --- collision BGK puis advection (schéma "push") ---
      for (int i = 0; i < kQ; ++i) {
        const double fi = m_f[i * N + n];
        const double fpost = fi - m_omega * (fi - feq_dir(i, rho, ux, uy));

        const int xn = x + kCx[i];
        const int yn = y + kCy[i];

        const bool out_of_domain = xn < 0 || yn < 0 || xn >= m_w || yn >= m_h;

        if (out_of_domain) {
          // Bord du domaine (paroi haut/bas, ou entrée/sortie réécrites après).
          m_f_tmp[kOpp[i] * N + n] = fpost;
        } else if (m_solid[idx(xn, yn)]) {
          // Heurte un obstacle : rebond complet + échange de quantité de
          // mouvement. La population arrive avec c_i*fpost et repart avec
          // -c_i*fpost ; l'obstacle encaisse donc 2*c_i*fpost.
          m_f_tmp[kOpp[i] * N + n] = fpost;
          fx += 2.0 * kCx[i] * fpost;
          fy += 2.0 * kCy[i] * fpost;
        } else {
          m_f_tmp[i * N + idx(xn, yn)] = fpost;
        }
      }
    }
  }
}

void LbmEngine::compute_macros() {
  const int N = m_w * m_h;
  for (int n = 0; n < N; ++n) {
    if (m_solid[n]) {
      m_rho[n] = 1.0;
      m_ux[n] = 0.0;
      m_uy[n] = 0.0;
      continue;
    }
    double rho = 0.0;
    double mx = 0.0;
    double my = 0.0;
    for (int i = 0; i < kQ; ++i) {
      const double fi = m_f[i * N + n];
      rho += fi;
      mx += kCx[i] * fi;
      my += kCy[i] * fi;
    }
    m_rho[n] = rho;
    m_ux[n] = mx / rho;
    m_uy[n] = my / rho;
  }
}

void LbmEngine::step(int sub_steps) {
  if (m_f.empty())
    return;

  const int iters = std::max(sub_steps, 1);
  double sum_fx = 0.0;
  double sum_fy = 0.0;
  for (int s = 0; s < iters; ++s) {
    apply_boundaries();
    double fx = 0.0;
    double fy = 0.0;
    collide_and_stream(fx, fy);
    m_f.swap(m_f_tmp);
    sum_fx += fx;
    sum_fy += fy;
  }

  // Effort moyen sur les sous-pas, puis lissage exponentiel (le signal oscille
  // au rythme du détachement tourbillonnaire).
  m_fx = sum_fx / iters;
  m_fy = sum_fy / iters;
  constexpr double kEma = 0.95;
  m_fx_ema = kEma * m_fx_ema + (1.0 - kEma) * m_fx;
  m_fy_ema = kEma * m_fy_ema + (1.0 - kEma) * m_fy;

  update_solid_bounds();
  compute_macros(); // une seule fois : sert uniquement au rendu
}

// ============================================================================
// Rendu
// ============================================================================
void LbmEngine::render_to_buffer(std::vector<Color> &pixel_buffer,
                                 float max_velocity_scale) {
  const int N = m_w * m_h;
  if (static_cast<int>(pixel_buffer.size()) != N)
    pixel_buffer.assign(N, Color{0, 0, 0, 255});

  // Vitesse mappée sur le haut de la colormap. <= 0 => échelle automatique.
  const double v_scale =
      max_velocity_scale > 0.0f ? max_velocity_scale : 1.8 * m_u_in;
  // Échelle de vorticité : ordre de grandeur u_in sur quelques cellules.
  const double w_scale = 0.6 * m_u_in;
  // Pression dynamique de référence : 1/2 rho_inf U_inf^2 (rho_inf = 1).
  const double q_dyn = 0.5 * m_u_in * m_u_in;

  for (int y = 0; y < m_h; ++y) {
    for (int x = 0; x < m_w; ++x) {
      const int n = idx(x, y);
      Rgb c;

      if (m_solid[n]) {
        c = Rgb{60, 62, 74};
      } else {
        switch (m_field) {
        case Field::Speed: {
          const double sp = std::sqrt(m_ux[n] * m_ux[n] + m_uy[n] * m_uy[n]);
          c = colormap_speed(sp / v_scale);
          break;
        }
        case Field::Vorticity: {
          // omega_z = d(uy)/dx - d(ux)/dy, différences centrées (bords
          // clampés).
          const int xm = std::max(x - 1, 0);
          const int xp = std::min(x + 1, m_w - 1);
          const int ym = std::max(y - 1, 0);
          const int yp = std::min(y + 1, m_h - 1);
          const double dvy_dx = 0.5 * (m_uy[idx(xp, y)] - m_uy[idx(xm, y)]);
          const double dux_dy = 0.5 * (m_ux[idx(x, yp)] - m_ux[idx(x, ym)]);
          c = colormap_diverging((dvy_dx - dux_dy) / w_scale);
          break;
        }
        case Field::Pressure: {
          // p = c_s^2 rho ; Cp = (p - p_inf) / q_dyn = (rho - 1) / (3 q_dyn).
          const double cp = (m_rho[n] - 1.0) / (kInvCs2 * q_dyn);
          c = colormap_diverging(cp);
          break;
        }
        }
      }

      pixel_buffer[n] = Color{c.r, c.g, c.b, 255};
    }
  }
}
