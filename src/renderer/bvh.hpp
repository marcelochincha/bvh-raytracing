#pragma once

#include <math/sr_math.hpp>
#include <renderer/sr_geometry.hpp> // AABB
#include <vector>
#include <unordered_map>
#include <cstddef>
#include <cstdint>

struct texture; // forward-declared; include sr_texture.hpp to use

// =============================================================================
// Bounding Volume Hierarchy (BVH) — acceleration structure for ray tracing.
//
// WHY: the naive ray tracer tests every ray against EVERY triangle: O(N) per
// ray, so a frame is O(pixels * N). A BVH groups triangles into a tree of
// nested axis-aligned boxes (AABBs). A ray first tests the cheap boxes and
// only descends into the ones it actually hits, skipping whole subtrees of
// triangles. That turns the per-ray cost from O(N) into ~O(log N) on average.
//
// HOW the tree is built: top-down with the Surface Area Heuristic (SAH). At
// each node we try to split its triangles into two groups so that the expected
// cost of tracing a ray through them is minimal. SAH estimates that cost from
// the surface area of each child box times how many triangles it holds (a ray
// is more likely to enter a bigger box). We evaluate candidate splits with
// "binning" (cheap, ~12 buckets per axis) instead of testing every triangle
// boundary, which keeps the build fast while staying close to optimal.
// =============================================================================

namespace bvh {

// A shadeable triangle: geometry the BVH needs + payload the shader needs.
struct Tri {
    vec3  v0, v1, v2;
    vec3  normal;           // face normal (used unless `smooth`)
    vec3  albedo;           // base color [0,1]
    float roughness = 0.5f; // [0,1] — 0=mirror, 1=fully matte
    float metallic  = 0.0f; // [0,1] — 0=dielectric, 1=metal (no diffuse)
    float ior       = 1.5f; // index of refraction (unused until BTDF added)
    // Per-vertex normals for smooth (Gouraud/Phong) shading. Flat geometry
    // leaves these zeroed + smooth=false and shades with `normal` (cheap);
    // smooth meshes (spheres, loaded OBJ) set smooth=true so the shader
    // interpolates n0/n1/n2 across the triangle.
    bool  smooth    = false;
    vec3  n0, n1, n2;
    vec3  emission  = vec3(0.0f, 0.0f, 0.0f); // emissive radiance (HDR)
    // Per-vertex UVs for texture mapping; tex==nullptr means use albedo color.
    vec2           uv0{}, uv1{}, uv2{};
    const texture* tex = nullptr;
};

// Result of the nearest-hit query.
struct Hit {
    float t   = 1e30f;   // distance along the ray to the closest hit
    int   tri = -1;      // index of the hit triangle (use BVH::tri(idx)), -1 = miss
};

// How the tree is split during construction. The build strategy trades build
// speed against traversal quality, which is the whole point of the comparison
// study: SAH (best tree, slower build), Median (cheap, decent), Morton (fastest
// build, lower-quality tree).
enum BuildStrategy { SAH = 0, Median = 1, Morton = 2 };

// Per-thread traversal counters. Each worker thread accumulates how many BVH
// nodes it tests while tracing; summing across threads yields the total node
// visits per frame -> average nodes/ray, a hardware-independent tree-quality
// metric. Uses thread_local storage so there is NO atomic on the hot path.
void reset_thread_node_visits();        // zero THIS thread's counter
long take_thread_node_visits();         // read & reset THIS thread's counter

class BVH {
public:
    // Build the tree over `tris` (consumes/moves the vector in). Triangles are
    // reordered internally so each leaf owns a contiguous range. `strategy`
    // chooses the split heuristic (SAH by default — the original implementation).
    void build(std::vector<Tri> tris, BuildStrategy strategy = SAH);

    BuildStrategy strategy() const { return strategy_; }

    // Nearest hit. Returns true and fills `out` if anything is hit; `out.t`
    // can be pre-set to clamp the search distance (defaults to +inf).
    bool intersect(const vec3& origin, const vec3& dir, Hit& out) const;

    // Any-hit, for shadow rays: returns true as soon as a triangle is hit
    // closer than `max_t` (no need to find the closest one).
    bool occluded(const vec3& origin, const vec3& dir, float max_t) const;

    const Tri&  tri(int i)          const { return tris_[i]; }
    std::size_t triangle_count()    const { return tris_.size(); }
    std::size_t node_count()        const { return nodes_.size(); }
    bool        empty()             const { return nodes_.empty(); }

    // --- GPU export (for OpenCL backend) ---
    // Flatten the tree into three parallel arrays consumable by the GPU kernel.
    // node_bounds: 8 floats/node (min.xyz + pad, max.xyz + pad as two float4).
    // node_links:  4 ints/node  (left, right, start, count; left<0 means leaf).
    // tris:        40 floats/tri (see bvh.cpp for the full layout). If `texmap`
    //              is non-null, slot [19] gets the texture atlas id (else -1) and
    //              slots [32-37] get the per-vertex UVs, so the GPU can sample.
    struct GPUAtlas {
        std::vector<uint32_t> pixels;   // all textures' ARGB pixels concatenated
        std::vector<int> off, w, h;     // per-texture (tex_id indexes these)
    };
    // Collect every unique texture referenced by this BVH's triangles into one
    // atlas (pixels + per-texture offset/width/height) and fill `texmap` so a
    // subsequent flatten(..., &texmap) can write the matching tex_id per tri.
    void build_atlas(GPUAtlas& out, std::unordered_map<const texture*, int>& texmap) const;
    void flatten(std::vector<float>& node_bounds,
                 std::vector<int>&   node_links,
                 std::vector<float>& tris,
                 const std::unordered_map<const texture*, int>* texmap = nullptr) const;

    // --- Debug visualization ---
    // One entry per node, with its box, its depth in the tree (root = 0) and
    // whether it's a leaf. Lets a caller draw the hierarchy as wireframe.
    struct DebugNode { AABB bounds; int depth; bool leaf; };
    void debug_nodes(std::vector<DebugNode>& out) const;

    // Trace one ray like intersect(), but ALSO record every node the traversal
    // pops off the stack, flagging whether the ray's box test passed (so it
    // descended) or failed (so the whole subtree was pruned). This is the data
    // behind the "ray path" demo: it shows the sequence of boxes a ray walks and
    // WHY it discards subtrees. Much slower than intersect() (it appends to a
    // vector) — meant for a handful of debug rays, never the render loop.
    // `visited` is appended to, not cleared, so several BVHs can share one list.
    struct VisitedNode { AABB bounds; bool leaf; bool box_hit; int order; };
    bool intersect_debug(const vec3& origin, const vec3& dir, Hit& out,
                         std::vector<VisitedNode>& visited) const;

private:
    struct Node {
        AABB bounds;
        int  left  = -1;   // interior: left child index; leaf: -1
        int  right = -1;   // interior: right child index
        int  start = 0;    // leaf: first triangle index
        int  count = 0;    // leaf: triangle count
    };

    std::vector<Tri>  tris_;
    std::vector<Node> nodes_;
    std::vector<vec3> centroids_;     // parallel to tris_, used only during build

    // Hard cap on tree depth. Top-down splitting can, in degenerate cases
    // (many coincident centroids), peel off one triangle at a time and build a
    // chain almost as deep as the triangle count. Traversal uses a fixed-size
    // stack, so an unbounded depth would overflow it and corrupt the C stack
    // (intermittent crashes). Capping the depth — forcing a leaf once we hit
    // it — keeps every traversal stack within MAX_DEPTH. A slightly fat leaf
    // just means a few extra triangle tests, which is harmless.
    static constexpr int MAX_DEPTH = 60;

    // Largest triangle count the SAH is allowed to leave in a single leaf. The
    // SAH normally decides leaf-vs-split on cost alone, but a few very large
    // triangles (a flat floor quad) can fool it into collapsing hundreds of
    // triangles into one leaf (see build_node). Past this size we force a median
    // split instead, keeping traversal fast regardless of such geometry.
    static constexpr int MAX_LEAF = 8;

    int  build_node(int start, int count, int depth);
    int  partition(int start, int count, int axis,
                   float cmin, float scale, int split_bin);
    // Reorder [start, start+count) so the first half holds the triangles with
    // the smallest centroid along `axis` (median split). Swaps tris_ and
    // centroids_ together. Returns the split index.
    int  partition_median(int start, int count, int axis);

    BuildStrategy strategy_ = SAH;
};

} // namespace bvh
