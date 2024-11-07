#include "mvla.h"

#include "boid.h"

/// Get the neighbourhood given side lengths through a boid's center
static rect_t neighbourhood_from_distances(boid_t boid, float width, float height);

boid_t boid_new(v2f_t position, v2f_t velocity) {
  boid_t boid;
  boid.position = position;
  boid.velocity = velocity;
  return boid;
}

float boid_sqr_distance(boid_t a, boid_t b) {
  return v2f_sqr_len(v2f_sub(a.position, b.position));
}

rect_t boid_neighbourhood(boid_t boid) {
  return neighbourhood_from_distances(boid, NEIGHBOURHOOD_WIDTH, NEIGHBOURHOOD_HEIGHT);
}

static rect_t neighbourhood_from_distances(boid_t boid, float width, float height) {
  float hw = width/2.0, hh = height/2.0;
  return rect_new(boid.position, hw, hh);
}