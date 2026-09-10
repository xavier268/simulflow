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

// Grille de simulation (cellules) — la texture est ensuite étirée à la fenêtre.
constexpr int kGridWidth = 320;
constexpr int kGridHeight = 200;

// Itérations LBM par image affichée. Monter = évolution plus rapide, coûte +
// cher.
constexpr int kSubSteps = 4;

constexpr int kBrushRadius = 4; // rayon du pinceau à obstacles (cellules)

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

  bool paused = false;

  // --- Boucle principale -----------------------------------------------
  while (!WindowShouldClose()) {
    // 1. Entrées clavier.
    if (IsKeyPressed(KEY_SPACE))
      paused = !paused;
    if (IsKeyPressed(KEY_R))
      engine->reset();
    if (IsKeyPressed(KEY_V)) {
      const int next = (static_cast<int>(engine->render_field()) + 1) % 3;
      engine->set_render_field(static_cast<LbmEngine::Field>(next));
    }

    // 2. Entrées souris : clic gauche = obstacle, clic droit = gomme.
    const Vector2 m = GetMousePosition();
    const int gx = static_cast<int>(m.x / kWindowWidth * kGridWidth);
    const int gy = static_cast<int>(m.y / kWindowHeight * kGridHeight);
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
      engine->stamp_disk(gx, gy, kBrushRadius, true);
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
      engine->stamp_disk(gx, gy, kBrushRadius, false);

    // 3. Simulation.
    if (!paused)
      engine->step(kSubSteps);

    // 4. Rendu moteur -> buffer -> texture GPU.
    engine->render_to_buffer(pixels, 0.0f);
    UpdateTexture(texture, pixels.data());

    // 5. Affichage.
    BeginDrawing();
    ClearBackground(BLACK);
    DrawTexturePro(texture, src, dst, {0, 0}, 0.0f, WHITE);

    // Efforts sur l'obstacle : flèche + coefficients.
    const LbmEngine::Bounds b = engine->solid_bounds();
    if (b.valid) {
      const float ox = (b.x0 + b.x1 + 1) * 0.5f / kGridWidth * kWindowWidth;
      const float oy = (b.y0 + b.y1 + 1) * 0.5f / kGridHeight * kWindowHeight;
      constexpr float kArrow = 6000.0f; // échelle purement visuelle
      const Vector2 base{ox, oy};
      const Vector2 tip{ox + static_cast<float>(engine->drag()) * kArrow,
                        oy - static_cast<float>(engine->lift()) * kArrow};
      DrawLineEx(base, tip, 2.0f, YELLOW);
      DrawCircleV(tip, 4.0f, YELLOW);
    }

    DrawText(TextFormat("%s   champ : %s%s", engine->get_name().c_str(),
                        field_name(engine->render_field()),
                        paused ? "   [PAUSE]" : ""),
             10, 10, 18, RAYWHITE);
    DrawText(TextFormat("Cx (trainee) = %+.3f    Cz (portance) = %+.3f",
                        engine->drag_coefficient(), engine->lift_coefficient()),
             10, 32, 18, YELLOW);
    DrawText("clic G : obstacle    clic D : gomme    V : champ    R : reset    "
             "Espace : pause",
             10, 54, 16, Fade(RAYWHITE, 0.7f));
    DrawFPS(kWindowWidth - 90, 10);
    EndDrawing();
  }

  // --- Nettoyage -------------------------------------------------------
  UnloadTexture(texture);
  CloseWindow();
  return 0;
}
