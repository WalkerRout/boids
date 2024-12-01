#include <stdlib.h>
#include <assert.h>

#include "rect.h"
#include "qtree.h"

/// Returns if a qtree has previously been subdivided
static bool is_subdivided(qtree_t *qtree);
/// Subdivide a qtree into its 4 quadrants
static void subdivide(qtree_t *qtree, arena_t *arena);
/// Query qtree within a given range, filling and growing found data as needed 
static void query_recursive(
  qtree_t *qtree, 
  rect_t range, 
  void ***found,
  size_t *found_count,
  size_t *found_capacity
);

qtree_t *qtree_new(
  arena_t *arena,
  size_t capacity,
  rect_t range,
  qtree_range_fn_t check_range
) {
  assert(arena != NULL);
  assert(capacity > 0);
  
  qtree_t *qtree = arena_alloc(arena, sizeof(qtree_t));
  assert(qtree != NULL);

  qtree->check_range = check_range;
  qtree->range = range;

  qtree->capacity = capacity;
  qtree->data_len = 0;
  qtree->data = arena_alloc(arena, capacity*sizeof(void *));
  assert(qtree->data != NULL);

  qtree->ne = NULL;
  qtree->se = NULL;
  qtree->sw = NULL;
  qtree->nw = NULL;

  return qtree;
}

bool qtree_insert(qtree_t *qtree, arena_t *arena, void *ele) {
  assert(qtree != NULL);

  if (!(qtree->check_range)(ele, qtree->range)) {
    return false;
  }

  if (qtree->data_len < qtree->capacity) {
    qtree->data[qtree->data_len++] = ele;
    return true;
  }

  if (!is_subdivided(qtree)) {
    subdivide(qtree, arena);
  }

  // Try inserting the element in one of the children
  return (qtree_insert(qtree->ne, arena, ele) ||
          qtree_insert(qtree->se, arena, ele) ||
          qtree_insert(qtree->sw, arena, ele) ||
          qtree_insert(qtree->nw, arena, ele));
}

void **qtree_query(qtree_t *qtree, rect_t query_range, size_t *out_count) {
  assert(qtree != NULL);

  size_t found_capacity = 16;
  void **found = calloc(found_capacity, sizeof(void *));
  assert(found != NULL);
  *out_count = 0;

  query_recursive(qtree, query_range, &found, out_count, &found_capacity);

  return found;
}

static bool is_subdivided(qtree_t *qtree) {
  assert(qtree != NULL);
  return qtree->ne != NULL;
}

static void subdivide(qtree_t *qtree, arena_t *arena) {
  assert(qtree != NULL);

  rect_t ne = {0}, se = {0}, sw = {0}, nw = {0};
  rect_quadrants(qtree->range, &ne, &se, &sw, &nw);

  qtree->ne = qtree_new(arena, qtree->capacity, ne, qtree->check_range);
  qtree->se = qtree_new(arena, qtree->capacity, se, qtree->check_range);
  qtree->sw = qtree_new(arena, qtree->capacity, sw, qtree->check_range);
  qtree->nw = qtree_new(arena, qtree->capacity, nw, qtree->check_range);
}

static void query_recursive(
  qtree_t *qtree, 
  rect_t range, 
  void ***found,
  size_t *found_count,
  size_t *found_capacity
) {
  assert(qtree != NULL);

  if (!rect_intersects(qtree->range, range)) {
    // we have nothing to check, return immediately
    return;
  }

  // are we entirely within the query, or should we only add some of our data?
  bool add_all = rect_is_inside(qtree->range, range);
  for (size_t i = 0; i < qtree->data_len; ++i) {
    if (add_all || (qtree->check_range)(qtree->data[i], range)) {
      // dynamic resize
      if (*found_count + 1 > *found_capacity) {
        *found_capacity *= 2;
        // *found = arena_realloc(arena, *found, *found_capacity * sizeof(void *));
        *found = realloc(*found, sizeof(void *) * (*found_capacity));
        assert(*found != NULL);
      }
      (*found)[(*found_count)++] = qtree->data[i];
    }
  }

  if (is_subdivided(qtree)) {
    // keep going...
    query_recursive(qtree->ne, range, found, found_count, found_capacity);
    query_recursive(qtree->se, range, found, found_count, found_capacity);
    query_recursive(qtree->sw, range, found, found_count, found_capacity);
    query_recursive(qtree->nw, range, found, found_count, found_capacity);
  }
}