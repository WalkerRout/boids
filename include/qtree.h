#ifndef QTREE_H
#define QTREE_H

#include <stddef.h>
#include <stdbool.h>

#include "rect.h"
#include "arena.h"

/// The function we inject to treat qtree dynamically
typedef bool (*qtree_range_fn_t)(void *ele, rect_t range);

/// A quadtree representing subdivided 2D space, storing some data per tree node
typedef struct qtree {
  qtree_range_fn_t check_range;
  rect_t range;

  size_t capacity;
  size_t data_len;
  void **data;

  struct qtree *ne;
  struct qtree *se;
  struct qtree *sw;
  struct qtree *nw;
} qtree_t;

/// Create a new qtree with the given capacity, range, and comparison function,
/// storing the memory for this node in an arena
qtree_t *qtree_new(
  arena_t *arena,
  size_t capacity,
  rect_t range,
  qtree_range_fn_t check_range
);

/// Insert an element into this tree, subdividing if full, returning success
bool qtree_insert(qtree_t *qtree, arena_t *arena, void *ele);

/// Get a list of all out_count elements in the tree falling into query_range
void **qtree_query(qtree_t *qtree, rect_t query_range, size_t *out_count);

#endif // QTREE_H