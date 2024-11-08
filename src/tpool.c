#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>

#include "tpool.h"

typedef struct work {
  thread_func_t func;
  void *arg;
  struct work *next;
} work_t;

struct tpool {
  // doubly linked work queue
  work_t *work_first;
  work_t *work_last;
  pthread_mutex_t work_queue_mutex;
  // signals the threads that there is work to be processed
  pthread_cond_t worker_cond;
  // signals the threads that the tpool has no threads processing
  pthread_cond_t working_cond;
  // track number of threads doing work
  size_t working_cnt;
  // track number of alive threads
  size_t thread_cnt;
  bool stop;
};

/// Initialize some unit of work to perform with a given task and data
static work_t *work_init(thread_func_t func, void *arg);
/// Free the memory of some unit of work
static void work_free(work_t *work);
/// Extract a chunk of work from tpool (need exclusive access)
static work_t *work_get(tpool_t *tp);
/// A perpetually running thread that manages work extraction and execution
static void *worker(void *arg);

tpool_t *tpool_init(size_t num) {
  if (num == 0) num = 2;

  // init self
  tpool_t *tp = calloc(1, sizeof(*tp));
  tp->thread_cnt = num;

  // init sync objects
  pthread_mutex_init(&tp->work_queue_mutex, NULL);
  pthread_cond_init(&tp->worker_cond, NULL);
  pthread_cond_init(&tp->working_cond, NULL);

  // init queue
  tp->work_first = NULL;
  tp->work_last = NULL;

  // create worker threads
  pthread_t thread;
  for (size_t i = 0; i < num; i++) {
    pthread_create(&thread, NULL, worker, tp);
    pthread_detach(thread);
  }

  return tp;
}

void tpool_free(tpool_t *tp) {
  if (tp == NULL) return;

  // mutex zone
  {
    pthread_mutex_lock(&tp->work_queue_mutex);
    // free all work in queue
    work_t *work = tp->work_first;
    while (work != NULL) {
      work_t *work_next = work->next;
      work_free(work);
      work = work_next;
    }
    tp->work_first = NULL;
    tp->work_last = NULL;
    tp->stop = true;

    pthread_cond_broadcast(&tp->worker_cond);
    pthread_mutex_unlock(&tp->work_queue_mutex);
  }

  tpool_wait(tp);

  pthread_mutex_destroy(&tp->work_queue_mutex);
  pthread_cond_destroy(&tp->worker_cond);
  pthread_cond_destroy(&tp->working_cond);

  free(tp);
}

bool tpool_add_work(tpool_t *tp, thread_func_t func, void *arg) {
  if (tp == NULL) return false;

  work_t *work = work_init(func, arg);
  if (work == NULL) return false;

  // mutex zone
  {
    pthread_mutex_lock(&tp->work_queue_mutex);
    // is queue empty?
    if (tp->work_first == NULL) {
      // add as first
      tp->work_first = work;
      tp->work_last = tp->work_first;
    } else {
      // set current work_last next to new work
      tp->work_last->next = work;
      // effectively shift current last down one and set current last to new
      // work
      tp->work_last = work;
    }
    // wake up all waiting workers
    pthread_cond_broadcast(&tp->worker_cond);
    // release mutex for workers to fight over (to which only a lucky one will
    // get the work, others get NULL)
    pthread_mutex_unlock(&tp->work_queue_mutex);
  }

  return true;
}

void tpool_wait(tpool_t *tp) {
  if (tp == NULL) return;

  // mutex zone
  {
    pthread_mutex_lock(&tp->work_queue_mutex);
    for (;;) {
      // is there work left?
      // is it alive (not stopped) with nonzero workers?
      // is it stopped with living threads?
      if (tp->work_first != NULL || (!tp->stop && tp->working_cnt != 0) ||
          (tp->stop && tp->thread_cnt != 0)) {
        // wait for one thread to finish, looping again once it does
        // - thread potentially stealing another piece of work from queue
        pthread_cond_wait(&tp->working_cond, &tp->work_queue_mutex);
      } else {
        // nah we good to exit
        break;
      }
    }
    pthread_mutex_unlock(&tp->work_queue_mutex);
  }
}

static work_t *work_init(thread_func_t func, void *arg) {
  if (func == NULL) return NULL;
  work_t *work;
  work = malloc(sizeof(*work));
  work->func = func;
  work->arg = arg;
  work->next = NULL;
  return work;
}

static void work_free(work_t *work) {
  free(work);
}

static work_t *work_get(tpool_t *tp) {
  if (tp == NULL)
    return NULL;

  // try taking first available chunk of work in queue
  work_t *work;
  work = tp->work_first;
  if (work == NULL)
    return NULL;

  // if work was only one in queue, set queue to empty state
  if (work->next == NULL) {
    tp->work_first = NULL;
    tp->work_last = NULL;
  } else {
    // work was not last in queue, point queue to next work
    tp->work_first = work->next;
  }

  return work;
}

static void *worker(void *arg) {
  if (arg == NULL) return NULL;

  tpool_t *tp = arg;

  for (;;) {
    work_t *work;
    // mutex zone
    {
      pthread_mutex_lock(&tp->work_queue_mutex);
      // wait while there is no work to process; loop prevents erroneous waking
      while (tp->work_first == NULL && !tp->stop)
        pthread_cond_wait(&tp->worker_cond, &tp->work_queue_mutex);
      // stop if requested, ***still holding lock***, so we can modify some stuff
      // outside the loop (this is only way to exit the loop)
      if (tp->stop) break;
      // retrieve first worker
      work = work_get(tp); // very possibly NULL
      tp->working_cnt++;
      pthread_mutex_unlock(&tp->work_queue_mutex);
    }

    // try to run worker task on thread after releasing lock, freeing worker
    // upon completion
    if (work != NULL) {
      work->func(work->arg);
      work_free(work);
    }

    // mutex zone
    {
      pthread_mutex_lock(&tp->work_queue_mutex);
      // worker finished above, decrement count
      tp->working_cnt--;
      // check if no threads are working, sending signal to threads waiting
      if (!tp->stop && tp->working_cnt == 0 && tp->work_first == NULL) {
        pthread_cond_signal(&tp->working_cond);
      }
      pthread_mutex_unlock(&tp->work_queue_mutex);
    }
  }

  // this thread is done, decrement thread count
  tp->thread_cnt--;
  // signal tpool_wait that a thread has exited
  pthread_cond_signal(&tp->working_cond);
  // unlock mutex after stopping
  pthread_mutex_unlock(&tp->work_queue_mutex);

  return NULL;
}