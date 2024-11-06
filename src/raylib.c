#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <time.h>

#include <raylib.h>

#define MVLA_IMPLEMENTATION
#include "mvla.h"
#undef  MVLA_IMPLEMENTATION

#include "boid.h"
#include "simulation.h"

#include "arena.h"

#define WIDTH 1600.0f
#define HEIGHT 1000.0f

#define BOID_WIDTH 8.0f
#define BOID_HEIGHT 16.0f
#define BOID_STILL_RADIUS 10.0f // when boid has no velocity, draw as circle
#define BOID_COLOUR RED

/// Draw a singular boid at its given position, facing in the direction of 
/// its normalized velocity
void draw_boid(boid_t boid) {
  float vlen = v2f_len(boid.velocity);
  if (vlen == 0.0) {
    DrawCircle(
      boid.position.x, boid.position.y, 
      BOID_STILL_RADIUS, 
      BOID_COLOUR
    );
    return;
  }

  /*
  ** given a triangle facing upwards:
  **
  **          A
  **         /^\
  **        / | \
  **       /  H  \
  **      /   |   \
  **     /    v    \
  **    B<----W---->C
  **
  ** where point A is the top, point B is the left, and point C is the right,
  ** we have height H and width W.
  **
  ** we want to find the positions for A, B, and C given the current position
  ** and **direction**.
  **
  ** suppose the position for a bird is directly in the centre of it's body,
  ** we can then find all positions by moving forward/backward and left/right
  */

  // direction the boid is facing
  v2f_t direction = v2f_div(boid.velocity, v2ff(vlen));
  // isoceles formula direction
  v2f_t forward = v2f_mul(direction, v2ff((2.0f/3.0f)*BOID_HEIGHT));
  v2f_t backward = v2f_mul(direction, v2ff((-1.0f/3.0f)*BOID_HEIGHT));
  // perpendicular offsets
  v2f_t tangent_direction = v2f(-direction.y, direction.x);
  v2f_t offset = v2f_mul(tangent_direction, v2ff(BOID_WIDTH/2.0f));
  
  // final vertices
  v2f_t tip = v2f_add(boid.position, forward);
  v2f_t left = v2f_sub(v2f_add(boid.position, backward), offset);
  v2f_t right = v2f_add(v2f_add(boid.position, backward), offset);

  DrawTriangle(
    (Vector2){ tip.x, tip.y },
    (Vector2){ left.x, left.y },
    (Vector2){ right.x, right.y },
    BOID_COLOUR
  );
}

/// Draw an entire simulation (all the boids..)
void draw_simulation(simulation_t *sim) {
  assert(sim != NULL);
  for (size_t i = 0; i < sim->boids_len; ++i) {
    draw_boid(sim->boids[i]);
  }
}

int main(int argc, char *argv[]) {
  (void) argc;
  (void) argv;

  srand(time(NULL));

  // create window
  InitWindow((int) WIDTH, (int) HEIGHT, "boids");
  SetTargetFPS(120);

  // create simulation
  const size_t boid_count = 1000;
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
    // advance the simulation
    simulation_tick(&sim, (float) dt);
    // draw the simulation
    BeginDrawing();
      ClearBackground(BLACK);
      DrawFPS(10, 10);
      draw_simulation(&sim);
    EndDrawing();
  }

  // close window
  CloseWindow();

  // free simulation
  simulation_free(&sim);

  return EXIT_SUCCESS;
}