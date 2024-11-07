#ifndef RECT_H
#define RECT_H

#include <stdbool.h>

#include "mvla.h"

typedef struct rect {
  v2f_t center;
  float half_width;
  float half_height;
} rect_t;

// Function prototypes
rect_t rect_new(float x, float y, float half_width, float half_height);
bool rect_contains_point(rect_t *rect, v2f_t *point);
bool rect_intersects(rect_t *rect, rect_t *other);
bool rect_is_inside(rect_t *rect, rect_t *other);
void rect_quadrants(rect_t *rect, rect_t *ne, rect_t *nw, rect_t *sw, rect_t *se);

#endif // RECT_H
