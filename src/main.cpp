#include "core/LbmEngine.hpp"
#include "version.hpp"

#include "raylib.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <print>
#include <vector>

namespace {

// Fenêtre (pixels écran).
constexpr int kWindowWidth = 960;
constexpr int kWindowHeight = 600;

// Grille de simulation (cellules). Ratio identique à la fenêtre (16:10) pour ne
// pas déformer. Plus fin = plus de détail, mais coût ~ proportionnel au nombre
// de cellules.
constexpr int kGridWidth = 480;
constexpr int kGridHeight = 300;

// Itérations LBM par image affichée. Monter = évolution plus rapide, coûte +
// cher.
constexpr int kSubSteps = 2;

// Rayon du pinceau (cellules) : valeur initiale + bornes pour la molette.
constexpr int kBrushMin = 1;
constexpr int kBrushMax = 40;
constexpr int kBrushInit = 6;

constexpr int kRotStepDeg = 1;        // incrément de rotation (touches + / -)
constexpr double kRotCooldown = 0.12; // s : anti-rebond des touches + / -

// Flèche d'effort : facteur unités-réseau -> pixels, et longueur max affichée.
constexpr float kForcePxPerUnit = 220.0f;
constexpr float kForceMaxPx = 190.0f;

constexpr int kHistLen = 320; // échantillons du tracé Cz(t) (1 par image)

// Paramètres physiques (unités réseau).
constexpr double kTau = 0.6; // relaxation BGK  ->  nu = (tau-0.5)/3
constexpr double kInletVelocity = 0.08;

const char *field_name(LbmEngine::Field f) {
  switch (f) {
  case LbmEngine::Field::Speed:
    return "vitesse";
  case LbmEngine::Field::Vorticity:
    return "vorticite";
  case LbmEngine::Field::Pressure:
    return "pression (Cp)";
  }
  return "?";
}

// Tracé temporel auto-échelonné (l'axe vertical inclut toujours 0).
void draw_plot(const std::vector<float> &hist, Rectangle box,
               const char *label) {
  DrawRectangleRec(box, Fade(BLACK, 0.55f));
  DrawRectangleLinesEx(box, 1.0f, Fade(RAYWHITE, 0.30f));
  DrawText(label, static_cast<int>(box.x) + 6, static_cast<int>(box.y) + 4, 14,
           RAYWHITE);
  if (hist.size() < 2)
    return;

  float lo = 0.0f;
  float hi = 0.0f;
  for (float v : hist) {
    lo = std::min(lo, v);
    hi = std::max(hi, v);
  }
  const float pad = std::max(0.05f, (hi - lo) * 0.12f);
  lo -= pad;
  hi += pad;

  const auto y_of = [&](float v) {
    return box.y + box.height - (v - lo) / (hi - lo) * box.height;
  };

  if (lo < 0.0f && hi > 0.0f) {
    const float yz = y_of(0.0f);
    DrawLineEx({box.x, yz}, {box.x + box.width, yz}, 1.0f,
               Fade(RAYWHITE, 0.35f));
  }
  for (std::size_t i = 1; i < hist.size(); ++i) {
    const float x1 =
        box.x + static_cast<float>(i - 1) / (kHistLen - 1) * box.width;
    const float x2 = box.x + static_cast<float>(i) / (kHistLen - 1) * box.width;
    DrawLineEx({x1, y_of(hist[i - 1])}, {x2, y_of(hist[i])}, 1.5f, YELLOW);
  }
  DrawText(TextFormat("%+.2f", hi), static_cast<int>(box.x + box.width) - 42,
           static_cast<int>(box.y) + 2, 12, Fade(RAYWHITE, 0.7f));
  DrawText(TextFormat("%+.2f", lo), static_cast<int>(box.x + box.width) - 42,
           static_cast<int>(box.y + box.height) - 14, 12, Fade(RAYWHITE, 0.7f));
}

// Micro-benchmark headless : mesure le débit du solveur (MLUPS).
int run_bench(LbmEngine &engine, int total_iters) {
  using clock = std::chrono::steady_clock;
  constexpr int kChunk = 50;

  const auto t0 = clock::now();
  for (int done = 0; done < total_iters; done += kChunk)
    engine.step(kChunk);
  const auto t1 = clock::now();

  const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  const double lups =
      static_cast<double>(engine.width()) * engine.height() * total_iters;
  std::println(
      "bench : {} iters en {:.0f} ms  ->  {:.3f} ms/iter, {:.1f} MLUPS",
      total_iters, ms, ms / total_iters, lups / (ms * 1e3));

  // Contrôle de stabilité : la vitesse max doit rester finie et ~ O(u_in).
  std::vector<Color> px;
  engine.render_to_buffer(px, 1.0f);
  double vmax = 0.0;
  bool finite = true;
  for (int y = 0; y < engine.height(); ++y)
    for (int x = 0; x < engine.width(); ++x) {
      const double s = engine.speed_at(x, y);
      finite = finite && std::isfinite(s);
      vmax = std::max(vmax, s);
    }
  std::println("stabilite : vmax = {:.4f} (u_in = {}), fini = {}", vmax,
               kInletVelocity, finite);
  std::println("efforts   : Cx = {:+.4f}   Cz = {:+.4f}",
               engine.drag_coefficient(), engine.lift_coefficient());
  return finite ? 0 : 1;
}

} // namespace

int main(int argc, char **argv) {
  std::println("simulflow v{}  ({}, {})", version::VERSION, version::GIT_HASH,
               version::COMPILER);

  // Moteur manipulé via l'interface ISimulationEngine (dispatch virtuel).
  auto engine = std::make_unique<LbmEngine>();
  engine->set_relaxation_time(kTau);
  engine->set_inlet_velocity(kInletVelocity);
  engine->init(kGridWidth, kGridHeight);

  std::println("Moteur : {}   Re ~ {:.0f}", engine->get_name(),
               engine->reynolds(kGridHeight / 5.0));

  if (argc > 1 && std::strcmp(argv[1], "--bench") == 0) {
    const int iters = (argc > 2) ? std::max(std::atoi(argv[2]), 50) : 2000;
    return run_bench(*engine, iters - iters % 50);
  }

  std::vector<Color> pixels(static_cast<std::size_t>(kGridWidth) * kGridHeight,
                            Color{0, 0, 0, 255});

  // --- Fenêtre + texture Raylib -----------------------------------------
  SetConfigFlags(FLAG_VSYNC_HINT);
  InitWindow(kWindowWidth, kWindowHeight, "simulflow - LBM D2Q9");
  SetTargetFPS(60);

  Image canvas = GenImageColor(kGridWidth, kGridHeight, BLACK);
  Texture2D texture = LoadTextureFromImage(canvas);
  UnloadImage(canvas);

  const Rectangle src{0, 0, static_cast<float>(kGridWidth),
                      static_cast<float>(kGridHeight)};
  const Rectangle dst{0, 0, static_cast<float>(kWindowWidth),
                      static_cast<float>(kWindowHeight)};

  // Facteur cellule -> pixel écran (identique en x et y, ratio conservé).
  const float cell_px = static_cast<float>(kWindowWidth) / kGridWidth;

  bool paused = false;
  int brush = kBrushInit;
  double last_rot_time = 0.0; // anti-rebond des touches de rotation
  std::vector<float> cz_hist; // historique du coefficient de portance
  cz_hist.reserve(kHistLen);

  // --- Boucle principale -----------------------------------------------
  while (!WindowShouldClose()) {
    // 1. Clavier : pause / reset / champ affiché / rotation des obstacles.
    if (IsKeyPressed(KEY_SPACE))
      paused = !paused;
    if (IsKeyPressed(KEY_R))
      engine->reset();
    if (IsKeyPressed(KEY_V)) {
      const int next = (static_cast<int>(engine->render_field()) + 1) % 3;
      engine->set_render_field(static_cast<LbmEngine::Field>(next));
    }
    // Rotation : '+' / '-' (indépendant de la disposition clavier via
    // GetCharPressed), pavé numérique en secours. Debounce : la répétition
    // automatique du clavier est écrasée à UN pas par fenêtre de kRotCooldown,
    // donc un appui bref = 1°, un appui maintenu ~ kRotStepDeg / kRotCooldown
    // °/s.
    int rot_dir = 0;
    for (int ch = GetCharPressed(); ch != 0; ch = GetCharPressed()) {
      if (ch == '+')
        rot_dir = +1;
      else if (ch == '-')
        rot_dir = -1;
    }
    if (IsKeyPressed(KEY_KP_ADD))
      rot_dir = +1;
    if (IsKeyPressed(KEY_KP_SUBTRACT))
      rot_dir = -1;
    if (rot_dir != 0 && GetTime() - last_rot_time >= kRotCooldown) {
      engine->rotate(rot_dir * kRotStepDeg);
      last_rot_time = GetTime();
    }

    // 2. Molette : taille du pinceau.
    const float wheel = GetMouseWheelMove();
    if (wheel != 0.0f)
      brush = std::clamp(brush + static_cast<int>(wheel), kBrushMin, kBrushMax);

    // 3. Souris : clic gauche = obstacle, clic droit = gomme.
    const Vector2 mouse = GetMousePosition();
    const int gx = static_cast<int>(mouse.x / kWindowWidth * kGridWidth);
    const int gy = static_cast<int>(mouse.y / kWindowHeight * kGridHeight);
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
      engine->stamp_disk(gx, gy, brush, true);
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
      engine->stamp_disk(gx, gy, brush, false);

    // 4. Simulation.
    if (!paused) {
      engine->step(kSubSteps);
      cz_hist.push_back(static_cast<float>(engine->lift_coefficient()));
      if (static_cast<int>(cz_hist.size()) > kHistLen)
        cz_hist.erase(cz_hist.begin());
    }

    // 5. Rendu moteur -> buffer -> texture GPU.
    engine->render_to_buffer(pixels, 0.0f);
    UpdateTexture(texture, pixels.data());

    // 6. Affichage.
    BeginDrawing();
    ClearBackground(BLACK);
    DrawTexturePro(texture, src, dst, {0, 0}, 0.0f, WHITE);

    // Aperçu du pinceau sous le curseur.
    DrawCircleLinesV(mouse, brush * cell_px, Fade(RAYWHITE, 0.6f));

    // Efforts sur l'obstacle : flèche (Cx horizontal, Cz vertical) partant du
    // centre de l'obstacle. Échelle linéaire, longueur bornée pour rester dans
    // le cadre — au-delà de kForceMaxPx la direction reste juste mais la
    // longueur sature.
    const LbmEngine::Bounds b = engine->solid_bounds();
    if (b.valid) {
      const float ox = (b.x0 + b.x1 + 1) * 0.5f / kGridWidth * kWindowWidth;
      const float oy = (b.y0 + b.y1 + 1) * 0.5f / kGridHeight * kWindowHeight;
      float vx = static_cast<float>(engine->drag()) * kForcePxPerUnit;
      float vy = -static_cast<float>(engine->lift()) * kForcePxPerUnit;
      const float len = std::sqrt(vx * vx + vy * vy);
      if (len > kForceMaxPx) {
        vx *= kForceMaxPx / len;
        vy *= kForceMaxPx / len;
      }
      const Vector2 base{ox, oy};
      const Vector2 tip{ox + vx, oy + vy};
      DrawLineEx(base, tip, 2.5f, YELLOW);
      DrawCircleV(tip, 4.0f, YELLOW);
    }

    DrawText(TextFormat("%s   champ : %s   angle : %+d deg%s",
                        engine->get_name().c_str(),
                        field_name(engine->render_field()),
                        engine->rotation_deg(), paused ? "   [PAUSE]" : ""),
             10, 10, 18, RAYWHITE);
    // Coefficients : 2 décimales (1 pour la finesse), valeurs cadrées à droite
    // sur des colonnes fixes pour que l'affichage ne "danse" pas.
    const double cx = engine->drag_coefficient();
    const double cz = engine->lift_coefficient();
    const auto val_right = [](const char *s, int right_x, int y) {
      DrawText(s, right_x - MeasureText(s, 18), y, 18, YELLOW);
    };
    DrawText("Cx", 10, 32, 18, YELLOW);
    val_right(TextFormat("%+.2f", cx), 95, 32);
    DrawText("Cz", 115, 32, 18, YELLOW);
    val_right(TextFormat("%+.2f", cz), 200, 32);
    DrawText("finesse Cz/Cx", 220, 32, 18, YELLOW);
    val_right(std::fabs(cx) > 5e-3 ? TextFormat("%+.1f", cz / cx) : "--", 400,
              32);
    DrawText(
        TextFormat("clic G : obstacle   clic D : gomme   molette : pinceau "
                   "(%d)   +/- : pivoter   V : champ   R : reset   Espace",
                   brush),
        10, 54, 16, Fade(RAYWHITE, 0.7f));

    draw_plot(cz_hist, Rectangle{10.0f, kWindowHeight - 130.0f, 340.0f, 120.0f},
              "Cz(t)");

    DrawFPS(kWindowWidth - 90, 10);
    EndDrawing();
  }

  // --- Nettoyage -------------------------------------------------------
  UnloadTexture(texture);
  CloseWindow();
  return 0;
}
