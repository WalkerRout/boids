#ifndef ARENA_H
#define ARENA_H

#include <stdint.h>

#define REGION_DEFAULT_CAPACITY (2048) // word count, (*8) to get byte count

/// A region of memory
typedef struct region {
  struct region *next;
  size_t capacity;
  size_t offset;
  uintptr_t data[];
} region_t;

/// An arena allocator containing a list of pages of memory
typedef struct arena {
  // track our list of regions
  region_t *beg;
  // track the current region we are operating on
  region_t *end;
} arena_t;

/// Initialize an arena
void arena_init(arena_t *arena);

/// Free all memory associated with this arena
void arena_free(arena_t *arena);

/// Try to allocate some memory in this arena
void *arena_alloc(arena_t *arena, size_t size_bytes);

/// Empty all regions in arena without releasing memory
void arena_clear(arena_t *arena);

#endif // ARENA_H