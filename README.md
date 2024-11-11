# boids
Boids flocking with separation, alignment, and cohesion rules.

Some simpler optimizations were implemented and are detailed below, the simulation can also be reset/randomized by pressing the R key.

** DISCLAIMER: THIS SIMULATION DEPENDS ON POSIX THREADS, IT MAY NOT RUN ON WINDOWS DEPENDING ON YOUR CONFIG (it will run with WSL2 and xrdp though)**

## Emergence
![Swarm slowly aligning](https://i.giphy.com/media/v1.Y2lkPTc5MGI3NjExbHBkdGtla2F1cWt4eXd3and5OGx6dzc5ZjNyNm9qN3E4cDVncW51YiZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/xEGEaOjpmPRvuPtyRe/giphy.gif)
![Aligned population in a line](https://i.giphy.com/media/v1.Y2lkPTc5MGI3NjExdnN4cDhtZjd6dWRmazJocHk3c21iejA1ODk3MG8yYjZ5aWFkNHk5NyZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/uDPSry9FxW6kWdGxHI/giphy.gif)

Individuals will automatically align themselves over time, based on the below rule set.

## Rule Set
The rules are adjusted by scaling their steer velocities (in simulation.c's calculate_acceleration), allowing different patterns to emerge. Additional rules based on local flockmates can be easily introduced, as it would just involve adding a new rule to `boid_update_t`... the current rules were chosen given their popularity and effectiveness, in that we can get a very good visualization of emergence with only the 3 rules chosen.

The boids flock towards the origin by default (top-left corner), wrapping and repeating.

#### Separation Rule
**Individuals steer away from local flockmates.**

Individuals want to maintain some semblance of personal space, so we don't get a huge overlapping line of boids...

#### Alignment Rule
**Individuals steer towards the average heading of local flockmates.**

We would like to make sure all the individuals are moving together, so this rule will assist in aligning the entire population as the changes propagate.

#### Cohesion Rule
**Individuals steer towards the average position of local flockmates.**

This is the core of the flocking; we want the boids to aggregate together, so we can emulate the behaviour of real-life birds.

## Setup
1. Install conan with `sudo pip3 install conan` (assuming you have pip3 and are using unix)
2. Mark the launch script executable with `chmod +x ./launch.sh`
3. Build and run the simulation with `sudo ./launch.sh doit` (open launch.sh for more detailed options...)
   - This should automatically install dependencies on your system (raylib)

## Optimizations
#### Swap buffers
Objects are stored between two buffers; a read-only buffer holding the generation we are advancing, and a write-only buffer filled with garbage we can overwrite with the new generation. The buffer references are swapped after a generation is fully processed, since our (now) old generation can become the garbage we overwrite in the next generations swap buffer.

#### Quadtree
> https://en.wikipedia.org/wiki/Quadtree
For each frame, we generate a read-only quadtree based on the current population's position. The quadtree helps constrain our search space from quadratic to linear time complexity, as querying the quadtree to find nearby neighbours takes approximately `O(log(n))` time; for all `n` individuals, the overall complexity becomes `O(n*log(n))`.

The quadtree stores lists of references (lists of a mere sizeof(uintptr_t)) to the last generation's memory (which is treated as read-only); it persists for less time than the last generation. Since we allocate and free lots of memory in a short period of time, it makes sense to use an arena to store quadtree nodes and their data.

Note: the implementation for the *quadtree* is called qtree.h/qtree.c. A *qtree* and *quadtree* are NOT the same data structure, but I didn't feel like typing out *quadtree* since it doesn't read as good.

#### Arena allocator
As mentioned, the arena helps us manage reusable memory, so we can clear the arena ("free" the memory) without actually deallocating anything since we plan to reuse the chunks of memory for the next frame. It also helps reduce some of the necessary code required to free contained data structures.

#### Threadpool
Since we process one buffer into the next sequentially, with no dependency on the previous written state, and all important data only needs to be read-only, we can perform the processing of buffer segments in parallel. We use a threadpool to process chunks of the population safely and efficiently.

Note: this threadpool implementation inconsistently reports `possibly lost: 272 bytes in 1 blocks` (possibly more bytes, always a constant multiple) when valgrind is used. However, valgrind just seems to be unable to track released memory after the main thread exits (via https://stackoverflow.com/a/75006436, which also details how to suppress these known "leaks")...
