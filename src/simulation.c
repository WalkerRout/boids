#include <stdlib.h>
#include <assert.h>

#include "mvla.h"

#include "simulation.h"

/// separation, alignment, and cohesion are all normalized to magnitude=1
typedef struct boid_update {
  v2f_t separation;
  v2f_t alignment;
  v2f_t cohesion;
} boid_update_t;

/// Update all boids in the simulation, storing in swap buffer
static void update_boids(simulation_t *sim, float dt);
/// Keep boids on screen, currently just reverse velocity
static void constrain_boids(simulation_t *sim);
/// Swap buffers, old content is now ready to be written over
static void swap_buffers(simulation_t *sim);
/// Place the src boid into dest, and adjust given other boids and their count
static void update_boid_into_swap(boid_t *dest, const boid_t *src, boid_t *boids, size_t boids_len, float delta_time);
/// Determine directional deltas for a boid
static boid_update_t calculate_deltas(boid_t boid, boid_t *boids, size_t boids_len);
/// Calculate the acceleration of a boid with the given deltas
static v2f_t calculate_acceleration(boid_update_t deltas);
/// Cap a's magnitude to mag if mag > 0, otherwise do nothing
static v2f_t limit_magnitude(v2f_t a, float mag);
/// Steer a boid in a desired direction
static v2f_t steer(v2f_t current_velocity, v2f_t desired);
/// Safe division, produce 0.0 where nan would be produced
static v2f_t safe_v2f_div(v2f_t a, v2f_t b);

void simulation_init(simulation_t *sim, float width, float height, size_t boids_len) {
  assert(sim != NULL);
  sim->ticks = 0;
  sim->width = width;
  sim->height = height;
  sim->boids_len = boids_len;
  sim->boids = calloc(boids_len, sizeof(boid_t));
  sim->boids_swap = calloc(boids_len, sizeof(boid_t));

  for (size_t i = 0; i < boids_len; ++i) {
    sim->boids[i].position.x = width*randf();
    sim->boids[i].position.y = height*randf();
    sim->boids[i].velocity.x = MAX_SPEED*randf();
    sim->boids[i].velocity.y = MAX_SPEED*randf();
  }
}

void simulation_free(simulation_t *sim) {
  assert(sim != NULL);
  free(sim->boids);
  free(sim->boids_swap);
}

void simulation_tick(simulation_t *sim, float dt) {
  assert(sim != NULL);
  // boids stores our current (last) generation
  // - we want to make a new generation
  // - we will store it in boids_swap
  // - so we write to boids_swap
  // - once we finish, we make boids point to boids_swap (the new written generation)
  //   and boids_swap point to boids (the last generation which we can overwrite now)
  update_boids(sim, dt);
  swap_buffers(sim);
  constrain_boids(sim);
  sim->ticks += 1;
}

static void update_boids(simulation_t *sim, float dt) {
  for (size_t i = 0; i < sim->boids_len; ++i) {
    update_boid_into_swap(
      &sim->boids_swap[i], &sim->boids[i],
      sim->boids, sim->boids_len,
      dt
    );
  }
}

static void constrain_boids(simulation_t *sim) {
  for (size_t i = 0; i < sim->boids_len; ++i) {
    boid_t *curr = &sim->boids[i];
    float cx = curr->position.x, cy = curr->position.y;
    if (cx < 0.0) {
      curr->position.x = sim->width;
    }
    if (cx > sim->width) {
      curr->position.x = 0;
    }
    if (cy < 0.0) {
      curr->position.y = sim->height;
    }
    if (cy > sim->height) {
      curr->position.y = 0;
    }
  }
}

static void swap_buffers(simulation_t *sim) {
  // store boids in temp and move next gen into boids
  boid_t *temp_boids = sim->boids;
  sim->boids = sim->boids_swap;
  sim->boids_swap = temp_boids;
}

static void update_boid_into_swap(boid_t *dest, const boid_t *src, boid_t *boids, size_t boids_len, float dt) {
  // now calculate deltas and update given acceleration
  boid_update_t update = calculate_deltas(*src, boids, boids_len);
  v2f_t acceleration = v2f_mul(calculate_acceleration(update), v2ff(dt));
  dest->velocity = limit_magnitude(v2f_add(src->velocity, acceleration), MAX_SPEED);
  dest->position = v2f_add(src->position, v2f_mul(src->velocity, v2ff(dt)));
}

static boid_update_t calculate_deltas(boid_t boid, boid_t *boids, size_t boids_len) {
  boid_update_t update = {0};
  size_t update_count = 0;
  for (size_t i = 0; i < boids_len; ++i, ++update_count) {
    boid_t other = boids[i];
    float cx = boid.position.x,  cy = boid.position.y,
          ox = other.position.x, oy = other.position.y;
    if (cx == ox && cy == oy) continue;
    // TODO: quadtree this whole step
    // determine neighbourhood of boid
    float dist = boid_sqr_distance(boid, other);
    if (dist < (HOOD_RADIUS*HOOD_RADIUS)) {
      // separation
      if (dist < (HOOD_RADIUS*HOOD_RADIUS)/3.0) {
        v2f_t diff = v2f_sub(boid.position, other.position);
        float mag_diff = v2f_len(diff);
        v2f_t norm_diff = safe_v2f_div(diff, v2ff(mag_diff));
        update.separation = v2f_add(update.separation, safe_v2f_div(norm_diff, v2ff(mag_diff)));
      }
      // alignment
      update.alignment = v2f_add(update.alignment, other.velocity);
      // cohesion
      update.cohesion = v2f_add(update.cohesion, other.position);
    }
  }

  if (update_count > 0) {
    // average over count
    update.separation = safe_v2f_div(update.separation, v2ff((float) update_count));
    update.alignment = safe_v2f_div(update.alignment, v2ff((float) update_count));
    update.cohesion = safe_v2f_div(update.cohesion, v2ff((float) update_count));

    // separation
    v2f_t norm_separation = safe_v2f_div(update.separation, v2ff(v2f_len(update.separation)));
    update.separation = steer(boid.velocity, v2f_mul(norm_separation, v2ff(MAX_SPEED)));

    // alignment
    update.alignment = v2f_sub(update.alignment, boid.position);
    update.alignment = steer(boid.velocity, v2f_mul(update.alignment, v2ff(MAX_SPEED)));

    // cohesion
    v2f_t norm_velocity = safe_v2f_div(boid.velocity, v2ff(v2f_len(boid.velocity)));
    update.cohesion = steer(boid.velocity, v2f_mul(norm_velocity, v2ff(MAX_SPEED)));
  }

  return update;
}

static v2f_t calculate_acceleration(boid_update_t deltas) {
  float separation_scale = 1.0;
  float alignment_scale = 2.0;
  float cohesion_scale = 5.0;
  // scale deltas (for customizing behaviour), default is a noop
  v2f_t sep = v2f_mul(deltas.separation, v2ff(separation_scale));
  v2f_t ali = v2f_mul(deltas.alignment, v2ff(alignment_scale));
  v2f_t coh = v2f_mul(deltas.cohesion, v2ff(cohesion_scale));
  return v2f_add(sep, v2f_add(ali, coh));
}

static v2f_t steer(v2f_t current_velocity, v2f_t desired) {
  return limit_magnitude(v2f_sub(desired, current_velocity), MAX_FORCE);
}

static v2f_t limit_magnitude(v2f_t a, float mag) {
  if (mag > 0.0 && v2f_sqr_len(a) > mag*mag) {
    a = v2f_mul(safe_v2f_div(a, v2ff(v2f_len(a))), v2ff(mag));
  }
  return a;
}

static v2f_t safe_v2f_div(v2f_t a, v2f_t b) {
  if (b.x == 0.0) {
    a.x = 0.0;
  } else {
    a.x /= b.x;
  }

  if (b.y == 0.0) {
    a.y = 0.0;
  } else {
    a.y /= b.y;
  }

  return a;
}