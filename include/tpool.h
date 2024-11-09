#ifndef TPOOL_H
#define TPOOL_H

//#include <stddef.h>

/// A function type we will use to represent a unit of work to perform in parallel
typedef void (*thread_func_t)(void *arg);

/// A fixed-size threadpool, implemented using pthreads
typedef struct tpool tpool_t;

/// Initalize a threadpool
tpool_t *tpool_new(size_t num);

/// Free a threadpool and all its threads, waiting on outstanding work
void tpool_free(tpool_t *tp);

/// Add some unit of work to this threadpools queue
bool tpool_add_work(tpool_t *tp, thread_func_t func, void *arg);

/// Wait for all work in the queue to finish
void tpool_wait(tpool_t *tp);

#endif // TPOOL_H
