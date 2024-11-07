#ifndef RECT_H
#define RECT_H

#include <stdbool.h>

#include "mvla.h"

/// A rectangle centered in 2D space with a half-width and half-height field
typedef struct rect {
  v2f_t center;
  float half_width;
  float half_height;
} rect_t;

/// You can guess what this function does..
rect_t rect_new(v2f_t center, float half_width, float half_height);

/// Determine if point intersects with the range covered by rect
bool rect_contains_point(rect_t rect, v2f_t point);

/// Do rect and other overlap?
bool rect_intersects(rect_t rect, rect_t other);

/// Is rect full contained within other?
bool rect_is_inside(rect_t rect, rect_t other);

/// Fill in ne/nw/sw/se with respective cartesian quadrants given this center 
/// and half dimensions
void rect_quadrants(rect_t rect, rect_t *ne, rect_t *nw, rect_t *sw, rect_t *se);

#endif // RECT_H
