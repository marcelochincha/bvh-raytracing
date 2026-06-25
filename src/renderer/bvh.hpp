#pragma once

#include <math/sr_math.hpp>
#include <renderer/sr_geometry.hpp> // AABB
#include <vector>
#include <cstddef>

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
    vec3  normal;        // face normal (used unless `smooth`)
    vec3  albedo;        // [0,1]
    float reflectivity;  // [0,1]
    // Per-vertex normals for smooth (Gouraud/Phong) shading. Flat geometry
    // leaves these zeroed + smooth=false and shades with `normal` (cheap);
    // smooth meshes (spheres, loaded OBJ) set smooth=true so the shader
    // interpolates n0/n1/n2 across the triangle.
    bool  smooth = false;
    vec3  n0, n1, n2;
};

// Result of the nearest-hit query.
struct Hit {
    float t   = 1e30f;   // distance along the ray to the closest hit
    int   tri = -1;      // index of the hit triangle (use BVH::tri(idx)), -1 = miss
};

class BVH {
public:
    // Build the tree over `tris` (consumes/moves the vector in). Triangles are
    // reordered internally so each leaf owns a contiguous range.
    void build(std::vector<Tri> tris);

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

    // --- Debug visualization ---
    // One entry per node, with its box, its depth in the tree (root = 0) and
    // whether it's a leaf. Lets a caller draw the hierarchy as wireframe.
    struct DebugNode { AABB bounds; int depth; bool leaf; };
    void debug_nodes(std::vector<DebugNode>& out) const;

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

    int  build_node(int start, int count, int depth);
    int  partition(int start, int count, int axis,
                   float cmin, float scale, int split_bin);
};

} // namespace bvh
