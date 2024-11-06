#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "arena.h"

void arena_init(arena_t *arena, ptrdiff_t capacity) {
  assert(arena != NULL);
  arena->offset = 0;
  arena->mem.beg = malloc(capacity);
  if (arena->mem.beg != NULL) {
    arena->mem.end = arena->mem.beg + capacity;
  } else {
    arena->mem.end = NULL;
  }
}

void *arena_alloc(arena_t *arena, ptrdiff_t size, ptrdiff_t align, ptrdiff_t count) {
  assert(arena != NULL);
  unsigned char *new_beg = arena->mem.beg + arena->offset;
  ptrdiff_t padding = -((uintptr_t) new_beg) & (align - 1);
  ptrdiff_t available = arena->mem.end - new_beg - padding;
  if (available < 0 || count > available/size) {
    return NULL;
  }
  void *p = new_beg + padding;
  arena->offset += padding + count*size;
  return memset(p, 0, count*size);
}

void arena_free(arena_t *arena) {
  assert(arena != NULL);
  free(arena->mem.beg);
}