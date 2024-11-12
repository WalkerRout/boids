#ifndef BOID_H
#define BOID_H

#include "mvla.h"

#include "rect.h"

#define NEIGHBOURHOOD_WIDTH  (40.0)
#define NEIGHBOURHOOD_HEIGHT (30.0)

/// A bird-oid object in the simulation, sitting at some position with some heading
typedef struct boid {
  v2f_t position;
  v2f_t velocity;
} boid_t;

/// Construct a new boid from position and velocity
boid_t boid_new(v2f_t position, v2f_t velocity);

/// Get the euclidean distance between two boids
float boid_sqr_distance(boid_t a, boid_t b);

/// Get the neighbourhood of detection for this boid (centered around position)
rect_t boid_neighbourhood(boid_t boid);

#endif // BOID_H