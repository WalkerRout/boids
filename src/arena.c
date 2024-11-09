#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "arena.h"

/// Create a new region of memory with a header containing capacity, offset, 
/// and the next region
static region_t *new_region(size_t capacity);
/// Free this region of memory
static void free_region(region_t *region);
/// Clear this region of memory without releasing memory
static void clear_region(region_t *region);

void arena_init(arena_t *arena) {
  assert(arena != NULL);
  arena->beg = NULL;
  arena->end = NULL;
}

void arena_free(arena_t *arena) {
  assert(arena != NULL);
  region_t *region = arena->beg;
  while (region) {
    region_t *next = region->next;
    free_region(region);
    region = next;
  }
  arena->beg = NULL;
  arena->end = NULL;
}

void *arena_alloc(arena_t *arena, size_t size_bytes) {
  assert(arena != NULL);
  // skip only past next boundary and normalize to its start
  size_t size = (size_bytes + sizeof(uintptr_t)-1)/sizeof(uintptr_t);

  if (arena->end == NULL) {
    size_t capacity = REGION_DEFAULT_CAPACITY;
    if (capacity < size) capacity = size;
    arena->end = new_region(capacity);
    arena->beg = arena->end;
  }

  while (arena->end->offset + size > arena->end->capacity && arena->end->next != NULL) {
    arena->end = arena->end->next;
  }

  assert(arena->end->next == NULL);

  if (arena->end->offset + size > arena->end->capacity) {
    // add new region at end
    size_t capacity = REGION_DEFAULT_CAPACITY;
    if (capacity < size) capacity = size;
    arena->end->next = new_region(capacity);
    arena->end = arena->end->next;
  }

  void *p = arena->end->data + arena->end->offset;
  arena->end->offset += size;
  return memset(p, 0, size);
}

void arena_clear(arena_t *arena) {
  assert(arena != NULL);
  for (region_t *curr = arena->beg; curr != NULL; curr = curr->next) {
    clear_region(curr);
  }
  arena->end = arena->beg;
}

static region_t *new_region(size_t capacity) {
  size_t bytes = sizeof(region_t) + capacity*sizeof(uintptr_t);
  region_t *region = calloc(bytes, 1);
  assert(region != NULL);
  region->next = NULL;
  region->capacity = capacity;
  region->offset = 0;
  return region;
}

static void free_region(region_t *region) {
  free(region);
}

static void clear_region(region_t *region) {
  region->offset = 0;
}
