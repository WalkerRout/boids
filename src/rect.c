#include "rect.h"

rect_t rect_new(float x, float y, float half_width, float half_height) {
  rect_t rect;
  rect.center = v2f(x, y);
  rect.half_width = half_width;
  rect.half_height = half_height;
  return rect;
}

bool rect_contains_point(rect_t *rect, v2f_t *point) {
  return (point->x >= rect->center.x - rect->half_width &&
          point->x <= rect->center.x + rect->half_width &&
          point->y >= rect->center.y - rect->half_height &&
          point->y <= rect->center.y + rect->half_height);
}

bool rect_intersects(rect_t *rect, rect_t *other) {
  return !(other->center.x - other->half_width > rect->center.x + rect->half_width ||
           other->center.x + other->half_width < rect->center.x - rect->half_width ||
           other->center.y - other->half_height > rect->center.y + rect->half_height ||
           other->center.y + other->half_height < rect->center.y - rect->half_height);
}

bool rect_is_inside(rect_t *rect, rect_t *other) {
  return (rect->center.x - rect->half_width >= other->center.x - other->half_width &&
          rect->center.x + rect->half_width <= other->center.x + other->half_width &&
          rect->center.y - rect->half_height >= other->center.y - other->half_height &&
          rect->center.y + rect->half_height <= other->center.y + other->half_height);
}

void rect_quadrants(rect_t *rect, rect_t *ne, rect_t *nw, rect_t *sw, rect_t *se) {
  float hw = rect->half_width / 2.0f;
  float hh = rect->half_height / 2.0f;

  ne->center = v2f(rect->center.x + hw, rect->center.y + hh);
  ne->half_width = hw;
  ne->half_height = hh;

  nw->center = v2f(rect->center.x - hw, rect->center.y + hh);
  nw->half_width = hw;
  nw->half_height = hh;

  sw->center = v2f(rect->center.x - hw, rect->center.y - hh);
  sw->half_width = hw;
  sw->half_height = hh;

  se->center = v2f(rect->center.x + hw, rect->center.y - hh);
  se->half_width = hw;
  se->half_height = hh;
}
