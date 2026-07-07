#pragma once
#include <cstdint>

// =============================================================================
// OpenCL ray-tracing offload — the GPU as a parallel co-processor.
//
// The CPU still owns everything (scene, logic, framebuffer, SDL present). This
// module traces the per-pixel rays on the GPU: upload the static field BVH
// once, the dynamic (floor + peds) BVH each frame, then run one work-item per
// pixel and read the colors back into the software framebuffer.
//
// All geometry is passed as flat float/int arrays (see BVH::flatten) so there
// is no struct-layout coupling between host C++ and the OpenCL kernel.
//
// If no OpenCL device is available, init() returns false and the caller falls
// back to the CPU thread-pool ray tracer — nothing else changes.
// =============================================================================
namespace ocl {

bool init(int max_ray_depth, float ambient, float shadow_eps);
bool available();

// Name of the compute device (e.g. "NVIDIA GeForce RTX 3060"), or "" if inactive.
const char* device_name();

// Upload the STATIC field BVH once (the city / sphere field). Pass nnodes==0
// and null pointers if the BVH is empty.
void set_room(const float* node_bounds, const int* node_links,
              const float* tris, int nnodes, int ntris);

// Upload the skybox cubemap once: all 6 faces' ARGB pixels concatenated, plus
// per-face start offset (in pixels), width and height (arrays of 6).
void set_sky(const uint32_t* pixels, int npx,
             const int* off, const int* w, const int* h);

// Upload the DYNAMIC scene BVH (rebuilt every frame: floor + pedestrians).
void set_dynamic(const float* node_bounds, const int* node_links,
                 const float* tris, int nnodes, int ntris);

// Upload the emissive (area-light) triangles used for Next Event Estimation.
// Same 32-float/tri layout as flatten(); the kernel reads v0/v1/v2, the face
// normal and the emission. Pass count==0 to fall back to sun+ambient lighting.
void set_emissive(const float* tris, int count);

// Trace the whole frame on the GPU and read the colors back into `out`
// (W*H ARGB8888). Camera basis is precomputed host-side: ax/ay/az are the
// camera X/Y/-Z axes in world space; tb=tan(fovx/2), ta=tb/aspect.
// `spp` = samples per pixel, `skybox` (0/1) toggles cubemap vs flat-ambient
// background, `reflections` (0/1) gates the specular bounce — mirroring the
// same knobs on the CPU trace_ray path.
void render(float cpx, float cpy, float cpz,
            float axx, float axy, float axz,
            float ayx, float ayy, float ayz,
            float azx, float azy, float azz,
            float tb, float ta,
            float sunx, float suny, float sunz,
            int spp, int skybox, int reflections,
            int W, int H, uint32_t* out);

} // namespace ocl
