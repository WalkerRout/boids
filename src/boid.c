#include "mvla.h"

#include "boid.h"

boid_t boid_new(v2f_t position, v2f_t velocity) {
  boid_t boid;
  boid.position = position;
  boid.velocity = velocity;
  return boid;
}

float boid_sqr_distance(boid_t a, boid_t b) {
  return v2f_sqr_len(v2f_sub(a.position, b.position));
}