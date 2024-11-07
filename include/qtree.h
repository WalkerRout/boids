#ifndef QTREE_H
#define QTREE_H

int (*qtree_in_range_fn_t)(void *ele, rect_t *range);

typedef struct qtree {
  qtree_in_range_fn_t check_in_range;

  rect_t boundary;
  bool divided;

  size_t capacity;
  size_t data_len;
  void **data;

  struct qtree *ne;
  struct qtree *se;
  struct qtree *sw;
  struct qtree *nw;

} qtree_t;

/// Create a new qtree with the given range
qtree_t *qtree_new();

/// Free this node's subdivisions if it is divided, then free its memory and itself
void qtree_free(qtree_t *qtree);

/// Insert an element into this tree, subdividing if full
void qtree_insert(qtree_t *qtree, void *ele);


/// Remove all elements from this qtree
void qtree_clear(qtree_t *qtree);

#endif // QTREE_H