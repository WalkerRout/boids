#ifndef SIMULATION_H
#define SIMULATION_H

#include "boid.h"
#include "arena.h"

#define HOOD_RADIUS 60.0f
#define MAX_SPEED 200.0f
#define MAX_FORCE 50.0f

/// A boids flocking simulation (rules for separation, alignment, cohesion)
typedef struct simulation {
  size_t ticks;
  arena_t arena;

  // dimensions for simulation
  float width, height;

  // fixed size list of boids
  size_t  boids_len;
  boid_t *boids;
  boid_t *boids_swap;
} simulation_t;

/// Initialize a simulation with boids_len randomly spawned boids
void simulation_init(simulation_t *sim, float width, float height, size_t boids_len);

/// Free a simulations memory
void simulation_free(simulation_t *sim);

/// Update a simulation by a single tick
void simulation_tick(simulation_t *sim, float delta_time);

#endif // SIMULATION_H