# boids
Boids flocking with separation, alignment, and cohesion rules.

Some simpler optimizations were implemented and are detailed below, the simulation can also be reset/randomized by pressing the R key.

## Emergence
![Swarm slowly aligning]([https://i.ibb.co/887W0DL/swarm.gif](https://i.gyazo.com/de675f100aad8b0aba98beaaba58f9c7.gif))
![Aligned population in a line]([https://i.ibb.co/887W0DL/line.gif](https://i.gyazo.com/c461f850456cd63a67cf26c0ebfc8731.gif))

Individuals will automatically align themselves over time, based on the below rule set.

## Rule Set
The rules are adjusted by scaling their steer velocities (in simulation.c's calculate_acceleration), which allows for emergence of different patterns

The boids flock towards the origin by default (top-left corner), wrapping and repeating.

##### Separation
Individuals steer away from local flockmates.
##### Alignment
Individuals steer towards the average heading of local flockmates.
##### Cohesion
Individuals steer towards the average position of local flockmates.

## Setup
1. Install conan with `sudo pip3 install conan` (assuming you have pip3 and are using unix, otherwise skip the sudo)
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
