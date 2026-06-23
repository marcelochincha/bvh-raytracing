# Raytracer with BVH

A CPU, multithreaded ray tracer. Its core data structure is a **BVH (Bounding
Volume Hierarchy)** built with the **Surface Area Heuristic (SAH)**, which lets it
trace a scene of hundreds of thousands of triangles at interactive speed.

Built on top of the **SR-LEC** software-rendering engine (rasterizer, mesh
loading, math, SDL). This repo covers the ray tracer and the BVH; for the base
engine see the original repository.

---

## Screenshots


| Rasterized | Raytrace + BVH |
|---|---|
| ![raster](docs/img/raster.png) | ![raytrace](docs/img/raytrace.png) |

| BVH (box wireframe) | Reflections + skybox |
|---|---|
| ![bvh](docs/img/bvh_debug.png) | ![reflect](docs/img/reflect.png) |

Scene: a scanned room (`res/textures/iscv2.ply`, ~383k triangles) with the
engine's procedural shapes inside. The ceiling is clipped so light gets in.

---

## The BVH

The naive ray tracer tests every ray against every triangle: `O(P · N)` per frame
for `P` pixels and `N` triangles.

The BVH groups triangles into a binary tree of nested AABBs. A ray tests the cheap
boxes first and only descends into the ones it hits, skipping whole subtrees.
Per-ray cost drops from `O(N)` to `~O(log N)`.

The split at each node uses the **SAH**: a larger box is more likely to be hit, so
a split's cost is estimated as `area(left)·count(left) + area(right)·count(right)`,
and the cheapest is chosen. We **bin** centroids into 12 buckets per axis instead
of testing every split — fast, near-optimal.

| Operation | Average | Worst case |
|---|---|---|
| Build | `O(N log N)` | `O(N²)` (bounded by `MAX_DEPTH`) |
| Memory | `O(N)` (≤ 2N−1 nodes) | same |
| Nearest hit (`intersect`) | `O(log N)` | `O(N)` |
| Shadow ray (`occluded`) | `O(log N)` | `O(N)` |
| Frame | `O(P · log N)` | — |

Full analysis and pseudocode: [docs/raytracer_bvh.tex](docs/raytracer_bvh.tex).

---

## Code

```
src/renderer/bvh.hpp          # interface + structs (Tri, Hit, Node)
src/renderer/bvh.cpp          # SAH-binned build + traversal
src/game/sr_game.cpp          # tracing, shadows, reflections, skybox, threads
src/renderer/sr_geometry.cpp  # .ply loading (room scan)
```

- **Build**: top-down binned SAH (12 bins), split axis = longest side of the
  centroid box. (`build_node`, `partition`)
- **Ray-triangle**: Möller–Trumbore. (`tri_hit`)
- **Ray-AABB**: slab test with distance early-out. (`aabb_hit`)
- **Traversal**: iterative, fixed stack of 64, depth capped at 60.
- **Two BVHs**: static (room, built once) and dynamic (animated shapes, rebuilt
  per frame); each ray queries both, nearest wins.
- **Threads**: persistent pool, one start semaphore per worker, one counting done
  semaphore.

---

## Setup

Supply your own SDL2 binaries (<https://github.com/libsdl-org/SDL/releases/>).

- `lib/` — SDL2 library files
- `third_party/SDL2/` — SDL2 headers (`.h`)
- `bin/` — executable; also put the SDL2 runtime here (`SDL2.dll` / `SDL2.so`)
- `src/` — renderer and ray tracer source
- `res/` — textures and models (includes `iscv2.ply`)
- `docs/` — paper (`.tex`) and images
- `Makefile` — defaults to Windows with `SDL2.dll` in `bin/`

```sh
make        # build
make run    # run
```

---

## Controls

- `TAB` — toggle rasterizer / raytrace
- `WASD` + mouse — camera
- `#define ENABLE_ROOM 0` in `sr_game.cpp` — skip loading the room

---

## References

1. J. Goldsmith, J. Salmon. *Automatic Creation of Object Hierarchies for Ray
   Tracing*. IEEE CG&A, 7(5), 1987.
2. D. MacDonald, K. Booth. *Heuristics for Ray Tracing Using Space Subdivision*.
   The Visual Computer, 6(3), 1990. — the SAH.
3. I. Wald. *On Fast Construction of SAH-based Bounding Volume Hierarchies*. IEEE
   Symposium on Interactive Ray Tracing, 2007. — binned construction.
4. T. Möller, B. Trumbore. *Fast, Minimum Storage Ray/Triangle Intersection*.
   Journal of Graphics Tools, 2(1), 1997.
5. A. Williams, S. Barrs, R. K. Morley, P. Shirley. *An Efficient and Robust
   Ray-Box Intersection Algorithm*. Journal of Graphics Tools, 2005.
6. M. Pharr, W. Jakob, G. Humphreys. *Physically Based Rendering* (4th ed.), 2023.
   Ch. 4.
