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

constexpr double kPi = 3.14159265358979323846;

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
  // std::clamp laisse passer NaN inchangé (les deux comparaisons sont
  // fausses) ; sans ce garde-fou, une divergence numérique du solveur
  // (rho -> 0 => ux = mx/rho = NaN) donnerait un cast UB vers int puis un
  // accès hors bornes de `stops`.
  if (!std::isfinite(t))
    t = 1.0; // divergence -> couleur saturée plutôt qu'un index invalide
  const double x = std::clamp(t, 0.0, 1.0) * 4.0;
  const int k = std::min(3, static_cast<int>(x));
  return lerp(stops[k], stops[k + 1], x - k);
}

// Divergente bleu <- sombre -> rouge (t in [-1, 1]).
// Sert à la vorticité (sens de rotation) ET à la pression
// (dépression/surpression).
inline Rgb colormap_diverging(double t) {
  // Même garde-fou que colormap_speed : NaN traverse std::clamp sans être
  // rejeté.
  if (!std::isfinite(t))
    t = 1.0;
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
  // c = corde de RÉFÉRENCE (indépendante de l'angle), rho_inf = 1.
  const double c = m_chord_ref > 0 ? m_chord_ref : 1;
  const double q = 0.5 * m_u_in * m_u_in * c;
  return q > 0.0 ? m_fx_ema / q : 0.0;
}

double LbmEngine::lift_coefficient() const {
  const double c = m_chord_ref > 0 ? m_chord_ref : 1;
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
  m_solid_ref.assign(N, 0);

  m_angle_deg = 0;
  m_pivot_x = m_w * 0.5;
  m_pivot_y = m_h * 0.5;

  reset();

  // Obstacle par défaut : profil d'aile NACA 4 chiffres cambré, placé dans le
  // premier tiers du domaine. Cambré => portance non nulle dès l'incidence 0.
  const int chord = std::max(8, m_w / 3);
  stamp_airfoil_ref(m_w / 5, m_h / 2, chord, true);
  rebuild_solid();
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
// Obstacles — tout est écrit dans le masque de RÉFÉRENCE (angle 0), puis
// m_solid est reconstruit par rebuild_solid() au prochain step().
// ============================================================================
void LbmEngine::to_reference(double gx, double gy, double &rx,
                             double &ry) const {
  // Masque affiché = référence tournée de +m_angle_deg autour du pivot.
  // Donc  référence = R(-angle) * (point - pivot) + pivot.
  const double a = m_angle_deg * kPi / 180.0;
  const double ca = std::cos(a);
  const double sa = std::sin(a);
  const double dx = gx - m_pivot_x;
  const double dy = gy - m_pivot_y;
  rx = m_pivot_x + ca * dx + sa * dy;
  ry = m_pivot_y - sa * dx + ca * dy;
}

void LbmEngine::set_obstacle(int grid_x, int grid_y, bool active) {
  double rx = 0.0;
  double ry = 0.0;
  to_reference(grid_x, grid_y, rx, ry);
  const int ix = static_cast<int>(std::lround(rx));
  const int iy = static_cast<int>(std::lround(ry));
  if (ix < 0 || iy < 0 || ix >= m_w || iy >= m_h)
    return;
  m_solid_ref[idx(ix, iy)] = active ? 1 : 0;
  m_dirty = true;
}

void LbmEngine::stamp_disk(int cx, int cy, int radius, bool active) {
  // Le centre (grille affichée) est ramené dans le repère de référence, où l'on
  // trace un disque propre — évite les trous de rééchantillonnage.
  double rcx = 0.0;
  double rcy = 0.0;
  to_reference(cx, cy, rcx, rcy);
  const int icx = static_cast<int>(std::lround(rcx));
  const int icy = static_cast<int>(std::lround(rcy));

  const int r2 = radius * radius;
  for (int y = icy - radius; y <= icy + radius; ++y) {
    for (int x = icx - radius; x <= icx + radius; ++x) {
      const int dx = x - icx;
      const int dy = y - icy;
      if (dx * dx + dy * dy <= r2 && x >= 0 && y >= 0 && x < m_w && y < m_h)
        m_solid_ref[idx(x, y)] = active ? 1 : 0;
    }
  }
  m_dirty = true;
}

void LbmEngine::rotate(int increment_deg) {
  m_angle_deg += increment_deg;
  while (m_angle_deg > 180)
    m_angle_deg -= 360;
  while (m_angle_deg <= -180)
    m_angle_deg += 360;
  m_dirty = true;
}

// Translation rigide du masque de référence : comme la rotation se fait
// autour du centroïde du masque de référence, décaler ce dernier de (dx, dy)
// décale le centroïde d'autant, donc le masque AFFICHÉ (rotation comprise)
// suit exactement le même décalage (dx, dy) quel que soit l'angle courant.
// Les cellules qui sortiraient de la grille sont simplement perdues.
void LbmEngine::translate(int dx, int dy) {
  if (dx == 0 && dy == 0)
    return;

  std::vector<std::uint8_t> shifted(m_solid_ref.size(), 0);
  for (int y = 0; y < m_h; ++y) {
    const int sy = y - dy;
    if (sy < 0 || sy >= m_h)
      continue;
    for (int x = 0; x < m_w; ++x) {
      const int sx = x - dx;
      if (sx < 0 || sx >= m_w)
        continue;
      shifted[idx(x, y)] = m_solid_ref[idx(sx, sy)];
    }
  }
  m_solid_ref.swap(shifted);
  m_dirty = true;
}

// NACA 4 chiffres (par défaut ~ NACA 2412). x_le = bord d'attaque (cellule),
// y_mid = ligne de référence, chord = corde en cellules. Repère écran y vers le
// bas : on inverse le signe pour que la cambrure bombe vers le haut.
void LbmEngine::stamp_airfoil_ref(int x_le, int y_mid, int chord, bool active) {
  if (chord < 4)
    return;
  constexpr double mc = 0.02; // cambrure max (fraction de corde)
  constexpr double pc = 0.40; // position de la cambrure max
  constexpr double tc = 0.12; // épaisseur max (fraction de corde)

  for (int gx = x_le; gx <= x_le + chord; ++gx) {
    const double xf = static_cast<double>(gx - x_le) / chord; // [0, 1]
    if (xf < 0.0 || xf > 1.0)
      continue;

    const double yt =
        5.0 * tc *
        (0.2969 * std::sqrt(xf) - 0.1260 * xf - 0.3516 * xf * xf +
         0.2843 * xf * xf * xf - 0.1036 * xf * xf * xf * xf); // TE fermé

    double yc = 0.0;
    if (xf < pc)
      yc = mc / (pc * pc) * (2.0 * pc * xf - xf * xf);
    else
      yc = mc / ((1.0 - pc) * (1.0 - pc)) *
           ((1.0 - 2.0 * pc) + 2.0 * pc * xf - xf * xf);

    const int gy_top = y_mid - static_cast<int>(std::ceil((yc + yt) * chord));
    const int gy_bot = y_mid - static_cast<int>(std::floor((yc - yt) * chord));
    for (int gy = gy_top; gy <= gy_bot; ++gy)
      if (gx >= 0 && gy >= 0 && gx < m_w && gy < m_h)
        m_solid_ref[idx(gx, gy)] = active ? 1 : 0;
  }
  m_dirty = true;
}

void LbmEngine::rebuild_solid() {
  m_dirty = false;

  // Pivot = centroïde du masque de référence ; corde = étendue en x.
  double sx = 0.0;
  double sy = 0.0;
  long count = 0;
  int rx0 = m_w;
  int rx1 = -1;
  for (int y = 0; y < m_h; ++y)
    for (int x = 0; x < m_w; ++x)
      if (m_solid_ref[idx(x, y)]) {
        sx += x;
        sy += y;
        ++count;
        rx0 = std::min(rx0, x);
        rx1 = std::max(rx1, x);
      }
  if (count > 0) {
    m_pivot_x = sx / count;
    m_pivot_y = sy / count;
    m_chord_ref = rx1 - rx0 + 1;
  } else {
    m_pivot_x = m_w * 0.5;
    m_pivot_y = m_h * 0.5;
    m_chord_ref = 1;
  }

  const double a = m_angle_deg * kPi / 180.0;
  const double ca = std::cos(a);
  const double sa = std::sin(a);

  for (int y = 0; y < m_h; ++y) {
    for (int x = 0; x < m_w; ++x) {
      // Rotation inverse (gather) : chaque cellule affichée va chercher sa
      // valeur dans le masque de référence -> pas de trou.
      const double dx = x - m_pivot_x;
      const double dy = y - m_pivot_y;
      const long ix = std::lround(m_pivot_x + ca * dx + sa * dy);
      const long iy = std::lround(m_pivot_y - sa * dx + ca * dy);

      const bool solid =
          ix >= 0 && iy >= 0 && ix < m_w && iy < m_h &&
          m_solid_ref[idx(static_cast<int>(ix), static_cast<int>(iy))] != 0;

      const int n = idx(x, y);
      if (m_solid[n] && !solid) {
        // Cellule qui redevient fluide : populations réamorcées à l'équilibre.
        m_rho[n] = 1.0;
        m_ux[n] = m_u_in;
        m_uy[n] = 0.0;
        equilibrium_at(n, 1.0, m_u_in, 0.0);
      }
      m_solid[n] = solid ? 1 : 0;
    }
  }

  update_solid_bounds();
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

  if (m_dirty)
    rebuild_solid();

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
