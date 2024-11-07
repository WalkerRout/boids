#ifndef ARENA_H
#define ARENA_H

#include <stddef.h>

/// A region of memory
/// TODO: convert to an actual flexible block of memory
typedef struct arena_region {
  unsigned char *beg;
  unsigned char *end;
} arena_region_t;

/// An arena allocator containing a list of pages of memory
/// TODO: beginning and end page pointers
typedef struct arena {
  ptrdiff_t offset;
  arena_region_t mem;
} arena_t;

/// Initialize an arena with a predefined capacity
void arena_init(arena_t *arena, ptrdiff_t capacity);

/// Try to allocate some memory in this arena, returning NULL if full
void *arena_alloc(arena_t *arena, ptrdiff_t size, ptrdiff_t align, ptrdiff_t count);

/// Free all memory associated with this arena
void arena_free(arena_t *arena);

// TODO: arena_clear(), which sets all region offset pointers to 0

#endif // ARENA_H