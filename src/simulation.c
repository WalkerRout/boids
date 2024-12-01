#include <stdlib.h>
#include <assert.h>

#include "mvla.h"

#include "qtree.h"
#include "simulation.h"

#define THREAD_COUNT (4)

/// Separation, alignment, and cohesion are all normalized to magnitude=1
typedef struct boid_update {
  v2f_t separation;
  v2f_t alignment;
  v2f_t cohesion;
} boid_update_t;

/// A unit of work to perform on another thread; pretty much a request to update
/// sim->boids_swap[start..end] given the state of qtree
typedef struct {
  // TODO: probably shouldnt be storing the whole sim, since we read/write from
  // different parts of it...
  simulation_t *sim;
  size_t start;
  size_t end;
  float dt;
  qtree_t *qtree;
} boid_chunk_task_t;

/// Update all boids in the simulation, storing in swap buffer
static void update_boids(simulation_t *sim, float dt);
/// Keep boids on screen, currently just reverse velocity
static void constrain_boids(simulation_t *sim);
/// Swap buffers, old content is now ready to be written over
static void swap_buffers(simulation_t *sim);
/// Place the src boid into dest, and adjust given other boids and their count
static void update_boid_into_swap(boid_t *dest, const boid_t src, qtree_t *qtree, float delta_time);
/// Determine directional deltas for a boid
static boid_update_t calculate_deltas(boid_t boid, qtree_t *qtree);
/// Calculate the acceleration of a boid with the given deltas
static v2f_t calculate_acceleration(boid_update_t deltas);
/// Cap a's magnitude to mag if mag > 0, otherwise do nothing
static v2f_t limit_magnitude(v2f_t a, float mag);
/// Steer a boid in a desired direction
static v2f_t steer(v2f_t current_velocity, v2f_t desired);
/// Safe division, produce 0.0 where nan would be produced
static v2f_t safe_v2f_div(v2f_t a, v2f_t b);
/// The qtree_range_fn_t used in a boid quadtree
static bool boid_in_range(void *ele, rect_t range);
/// The thread_func_t work we want to do to update a range of boids into boids_swap
static void chunk_boid_update(void *arg);

void simulation_init(simulation_t *sim, float width, float height, size_t boids_len) {
  assert(sim != NULL);
  sim->ticks = 0;
  
  arena_init(&sim->arena);

  sim->pool = tpool_new(THREAD_COUNT);

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
  arena_free(&sim->arena);
  tpool_free(sim->pool);
}

void simulation_tick(simulation_t *sim, float dt) {
  assert(sim != NULL);
  update_boids(sim, dt);
  constrain_boids(sim);
  sim->ticks += 1;
}

static void update_boids(simulation_t *sim, float dt) {
  assert(sim != NULL);
  // initialize our quadtree
  float hw = sim->width/2.0, hh = sim->height/2.0;
  rect_t sim_range = rect_new(v2f(hw, hh), hw, hh);
  qtree_t *qtree = qtree_new(&sim->arena, 85, sim_range, boid_in_range);
  
  for (size_t i = 0; i < sim->boids_len; ++i) {
    qtree_insert(qtree, &sim->arena, (void *) &sim->boids[i]);
  }

  // chunk up population and pick up slack
  size_t chunk_size = sim->boids_len / THREAD_COUNT;
  size_t slack = sim->boids_len % THREAD_COUNT;
  boid_chunk_task_t tasks[THREAD_COUNT] = {0};

  size_t curr = 0;
  for (size_t i = 0; i < THREAD_COUNT; ++i) {
    size_t start = curr;
    size_t end = start + chunk_size;
    if (i < slack) {
      // add additional boids over first slack threads
      end += 1;
    }

    tasks[i].sim = sim;
    tasks[i].start = start;
    tasks[i].end = end;
    tasks[i].dt = dt;
    tasks[i].qtree = qtree;

    // add unit of work to threadpool
    tpool_add_work(sim->pool, chunk_boid_update, &tasks[i]);

    curr = end;
  }

  // finish updating
  tpool_wait(sim->pool);
  // reset arena/free quadtree
  arena_clear(&sim->arena);
  // swap buffers
  swap_buffers(sim);
}

static void constrain_boids(simulation_t *sim) {
  assert(sim != NULL);
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
  assert(sim != NULL);
  // store boids in temp and move next gen into boids
  boid_t *temp_boids = sim->boids;
  sim->boids = sim->boids_swap;
  sim->boids_swap = temp_boids;
}

static void update_boid_into_swap(boid_t *dest, const boid_t src, qtree_t *qtree, float dt) {
  assert(dest != NULL);
  // now calculate deltas and update given acceleration
  boid_update_t update = calculate_deltas(src, qtree);
  v2f_t acceleration = v2f_mul(calculate_acceleration(update), v2ff(dt));
  dest->velocity = limit_magnitude(v2f_add(src.velocity, acceleration), MAX_SPEED);
  dest->position = v2f_add(src.position, v2f_mul(src.velocity, v2ff(dt)));
}

static boid_update_t calculate_deltas(boid_t boid, qtree_t *qtree) {
  // initially we have deltas of 0
  boid_update_t update = {0};

  rect_t neighbourhood = boid_neighbourhood(boid);
  size_t neighbours_len = 0;
  boid_t **neighbours = (boid_t **) qtree_query(qtree, neighbourhood, &neighbours_len);

  size_t update_count = 0;
  for (size_t i = 0; i < neighbours_len; ++i, ++update_count) {
    boid_t other = *neighbours[i];

    if (boid.position.x == other.position.x && boid.position.y == other.position.y) {
      continue;
    }

    // separation
    float dist = boid_sqr_distance(boid, other);
    if (dist < (NEIGHBOURHOOD_WIDTH * NEIGHBOURHOOD_HEIGHT) / 9.0) {
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

  // we are done with our search
  free(neighbours);

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
  float separation_scale = 2.0;
  float alignment_scale = 2.0;
  float cohesion_scale = 3.0;
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
  a.x = (b.x == 0.0) ? 0.0 : a.x / b.x;
  a.y = (b.y == 0.0) ? 0.0 : a.y / b.y;
  return a;
}

static bool boid_in_range(void *ele, rect_t range) {
  assert(ele != NULL);
  boid_t *boid = (boid_t *) ele;
  return rect_contains_point(range, boid->position);
}

static void chunk_boid_update(void *arg) {
  assert(arg != NULL);
  boid_chunk_task_t *task = (boid_chunk_task_t *)arg;
  simulation_t *sim = task->sim;
  size_t start = task->start;
  size_t end = task->end;
  qtree_t *qtree = task->qtree;
  float dt = task->dt;

  for (size_t i = start; i < end; ++i) {
    update_boid_into_swap(&sim->boids_swap[i], sim->boids[i], qtree, dt);
  }
}
