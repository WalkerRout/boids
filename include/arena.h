#ifndef ARENA_H
#define ARENA_H

#include <stddef.h>

typedef struct arena_region {
  unsigned char *beg;
  unsigned char *end;
} arena_region_t;

typedef struct arena {
  ptrdiff_t offset;
  arena_region_t mem;
} arena_t;

void arena_init(arena_t *arena, ptrdiff_t capacity);
void *arena_alloc(arena_t *arena, ptrdiff_t size, ptrdiff_t align, ptrdiff_t count);
void arena_free(arena_t *arena);

#endif // ARENA_H