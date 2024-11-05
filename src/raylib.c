#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <raylib.h>

#define MVLA_IMPLEMENTATION
#include "mvla.h"
#undef  MVLA_IMPLEMENTATION

#include "boid.h"
#include "simulation.h"

#define WIDTH 1400.0
#define HEIGHT 1000.0

#define BOID_WIDTH 8.0
#define BOID_HEIGHT 16.0
#define BOID_STILL_RADIUS 10.0 // when boid has no velocity, draw as circle
#define BOID_COLOUR RED

/// Draw a singular boid at its given position, facing in the direction of 
/// its normalized velocity

void draw_boid(boid_t boid) {
  float vlen = v2f_len(boid.velocity);
  if (vlen == 0.0f) {
    DrawCircle(boid.position.x, boid.position.y, BOID_STILL_RADIUS, BOID_COLOUR);
    return;
  }

  // determine directionality
  v2f_t dir = v2f_div(boid.velocity, v2ff(vlen));
  v2f_t perp = v2f(-dir.y, dir.x);

  // construct triangle in space
  v2f_t front = v2f(BOID_HEIGHT/2.0, 0.0); // front
  v2f_t left = v2f(-BOID_HEIGHT/2.0, -BOID_WIDTH/2.0); // left base
  v2f_t right = v2f(-BOID_HEIGHT/2.0, BOID_WIDTH/2.0); // right base

  // rotate/translate/scale triangle in space
  v2f_t pa = v2f_add(
    boid.position,
    v2f_add(v2f_mul(dir, v2ff(front.x)), v2f_mul(perp, v2ff(front.y)))
  );
  v2f_t pb = v2f_add(
    boid.position,
    v2f_add(v2f_mul(dir, v2ff(left.x)), v2f_mul(perp, v2ff(left.y)))
  );
  v2f_t pc = v2f_add(
    boid.position,
    v2f_add(v2f_mul(dir, v2ff(right.x)), v2f_mul(perp, v2ff(right.y)))
  );

  DrawTriangle(
    (Vector2){ pa.x, pa.y },
    (Vector2){ pb.x, pb.y },
    (Vector2){ pc.x, pc.y },
    BOID_COLOUR
  );
}

/// Draw an entire simulation (all the boids..)
void draw_simulation(simulation_t *sim) {
  BeginDrawing();
    ClearBackground(BLACK);
    for (size_t i = 0; i < sim->boids_len; ++i) {
      draw_boid(sim->boids[i]);
    }
  EndDrawing();
}

int main(int argc, char *argv[]) {
  (void) argc;
  (void) argv;

  srand(time(NULL));

  // create window
  InitWindow(WIDTH, HEIGHT, "boids");
  SetTargetFPS(120);

  // create simulation
  const size_t boid_count = 500;
  simulation_t sim = {0};
  simulation_init(&sim, WIDTH, HEIGHT, boid_count);

  // run simulation
  while (!WindowShouldClose()) {
    double dt = GetFrameTime();
    // if user pressed r, reload the simulation
    if (IsKeyPressed(KEY_R)) {
      simulation_free(&sim);
      simulation_init(&sim, WIDTH, HEIGHT, boid_count);
    }
    simulation_tick(&sim, (float) dt);
    draw_simulation(&sim);
  }

  // close window
  CloseWindow();

  // free simulation
  simulation_free(&sim);

  return EXIT_SUCCESS;
}