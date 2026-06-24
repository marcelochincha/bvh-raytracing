#include <renderer/bvh.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace bvh {

// ---- small AABB helpers ----------------------------------------------------

static inline AABB aabb_empty() {
    return AABB{ vec3( 1e30f,  1e30f,  1e30f),
                vec3(-1e30f, -1e30f, -1e30f) };
}

static inline void aabb_grow(AABB& b, const vec3& p) {
    b.min = minimum(b.min, p);
    b.max = maximum(b.max, p);
}

static inline AABB aabb_union(const AABB& a, const AABB& b) {
    return AABB{ minimum(a.min, b.min), maximum(a.max, b.max) };
}

// Surface area of the box (the "SA" the SAH is named after). Half of it,
// actually — the constant factor cancels out when comparing split costs.
static inline float aabb_area(const AABB& b) {
    vec3 d = b.max - b.min;
    if (d.x < 0.0f) return 0.0f; // empty
    return d.x * d.y + d.y * d.z + d.z * d.x;
}

// AABB of a single triangle.
static inline AABB tri_bounds(const Tri& t) {
    AABB b = aabb_empty();
    aabb_grow(b, t.v0);
    aabb_grow(b, t.v1);
    aabb_grow(b, t.v2);
    return b;
}

// ---- ray primitives --------------------------------------------------------

// Slab test: does the ray hit the box within [0, t_max]? Returns the entry
// distance in `t_near` so traversal can skip boxes that are farther than the
// closest hit found so far.
static inline bool aabb_hit(const AABB& b, const vec3& o, const vec3& inv,
                            float t_max, float& t_near) {
    float tmin = 0.0f, tmax = t_max;
    for (int i = 0; i < 3; ++i) {
        float t0 = (b.min.v[i] - o.v[i]) * inv.v[i];
        float t1 = (b.max.v[i] - o.v[i]) * inv.v[i];
        if (inv.v[i] < 0.0f) std::swap(t0, t1);
        tmin = t0 > tmin ? t0 : tmin;
        tmax = t1 < tmax ? t1 : tmax;
        if (tmax < tmin) return false;
    }
    t_near = tmin;
    return true;
}

// Moller-Trumbore ray-triangle intersection.
static inline bool tri_hit(const Tri& tr, const vec3& o, const vec3& d, float& t) {
    const float EPS = 1e-8f;
    vec3 e1 = tr.v1 - tr.v0;
    vec3 e2 = tr.v2 - tr.v0;
    vec3 h  = cross(d, e2);
    float a = dot(e1, h);
    if (std::fabs(a) < EPS) return false;
    float f = 1.0f / a;
    vec3 s  = o - tr.v0;
    float u = f * dot(s, h);
    if (u < 0.0f || u > 1.0f) return false;
    vec3 q  = cross(s, e1);
    float v = f * dot(d, q);
    if (v < 0.0f || u + v > 1.0f) return false;
    t = f * dot(e2, q);
    return t > EPS;
}

// ---- build -----------------------------------------------------------------

void BVH::build(std::vector<Tri> tris) {
    tris_  = std::move(tris);
    nodes_.clear();
    if (tris_.empty()) return;

    centroids_.resize(tris_.size());
    for (std::size_t i = 0; i < tris_.size(); ++i)
        centroids_[i] = (tris_[i].v0 + tris_[i].v1 + tris_[i].v2) / 3.0f;

    nodes_.reserve(tris_.size() * 2);   // upper bound on node count
    build_node(0, (int)tris_.size(), 0);
    centroids_.clear();
    centroids_.shrink_to_fit();
}

// Partition triangles [start, start+count) so that those whose centroid falls
// in bins [0, split_bin] come first. Returns the index of the first triangle
// on the right side. Swaps both tris_ and centroids_ to keep them aligned.
int BVH::partition(int start, int count, int axis,
                   float cmin, float scale, int split_bin) {
    int i = start;
    int j = start + count - 1;
    while (i <= j) {
        int bin = (int)((centroids_[i].v[axis] - cmin) * scale);
        if (bin > split_bin) {           // belongs on the right -> swap to end
            std::swap(tris_[i], tris_[j]);
            std::swap(centroids_[i], centroids_[j]);
            --j;
        } else {
            ++i;
        }
    }
    return i; // first index on the right side
}

int BVH::build_node(int start, int count, int depth) {
    int idx = (int)nodes_.size();
    nodes_.push_back(Node{});

    // Node bounds = union of all its triangles' bounds.
    AABB bounds = aabb_empty();
    for (int i = start; i < start + count; ++i)
        bounds = aabb_union(bounds, tri_bounds(tris_[i]));

    // Leaf if small enough, or if we've hit the depth cap (keeps the tree
    // shallow enough that the fixed-size traversal stack never overflows).
    if (count <= 2 || depth >= MAX_DEPTH) {
        nodes_[idx].bounds = bounds;
        nodes_[idx].left = -1;
        nodes_[idx].start = start;
        nodes_[idx].count = count;
        return idx;
    }

    // Centroid bounds drive the split axis & bin range.
    AABB cb = aabb_empty();
    for (int i = start; i < start + count; ++i)
        aabb_grow(cb, centroids_[i]);

    vec3 ext = cb.max - cb.min;
    int axis = (ext.x > ext.y && ext.x > ext.z) ? 0 : (ext.y > ext.z ? 1 : 2);
    float cmin = cb.min.v[axis];
    float cext = ext.v[axis];

    // Degenerate spread (all centroids coincide on this axis) -> leaf.
    if (cext < 1e-8f) {
        nodes_[idx].bounds = bounds;
        nodes_[idx].left = -1;
        nodes_[idx].start = start;
        nodes_[idx].count = count;
        return idx;
    }

    // --- Binned SAH ---
    constexpr int NB = 12;
    AABB bin_bounds[NB];
    int  bin_count[NB] = {0};
    for (int i = 0; i < NB; ++i) bin_bounds[i] = aabb_empty();

    float scale = NB / cext;
    for (int i = start; i < start + count; ++i) {
        int b = (int)((centroids_[i].v[axis] - cmin) * scale);
        if (b < 0) b = 0; if (b >= NB) b = NB - 1;
        bin_bounds[b] = aabb_union(bin_bounds[b], tri_bounds(tris_[i]));
        bin_count[b]++;
    }

    // Sweep to get, for each of the NB-1 split planes, the area*count of the
    // left side and the right side.
    float left_area[NB - 1], right_area[NB - 1];
    int   left_cnt[NB - 1],  right_cnt[NB - 1];

    AABB acc = aabb_empty(); int cnt = 0;
    for (int i = 0; i < NB - 1; ++i) {
        acc = aabb_union(acc, bin_bounds[i]);
        cnt += bin_count[i];
        left_area[i] = aabb_area(acc);
        left_cnt[i]  = cnt;
    }
    acc = aabb_empty(); cnt = 0;
    for (int i = NB - 1; i > 0; --i) {
        acc = aabb_union(acc, bin_bounds[i]);
        cnt += bin_count[i];
        right_area[i - 1] = aabb_area(acc);
        right_cnt[i - 1]  = cnt;
    }

    // Pick the split with the lowest SAH cost.
    float best_cost = 1e30f;
    int   best_split = -1;
    for (int i = 0; i < NB - 1; ++i) {
        float cost = left_area[i] * left_cnt[i] + right_area[i] * right_cnt[i];
        if (cost < best_cost) { best_cost = cost; best_split = i; }
    }

    // Compare against the cost of NOT splitting (making a leaf). The 1.0f is a
    // traversal-step constant; parent area normalizes the split cost.
    float parent_area = aabb_area(bounds);
    float split_cost  = 1.0f + best_cost / (parent_area > 0.0f ? parent_area : 1.0f);
    float leaf_cost   = (float)count;
    if (best_split < 0 || split_cost >= leaf_cost) {
        nodes_[idx].bounds = bounds;
        nodes_[idx].left = -1;
        nodes_[idx].start = start;
        nodes_[idx].count = count;
        return idx;
    }

    // Partition in place; fall back to a median split if binning degenerates.
    int mid = partition(start, count, axis, cmin, scale, best_split);
    if (mid == start || mid == start + count)
        mid = start + count / 2;

    int l = build_node(start, mid - start, depth + 1);
    int r = build_node(mid, start + count - mid, depth + 1);

    // NOTE: nodes_ may have reallocated during recursion, so index by `idx`.
    nodes_[idx].bounds = bounds;
    nodes_[idx].left  = l;
    nodes_[idx].right = r;
    nodes_[idx].count = 0;
    return idx;
}

// ---- traversal -------------------------------------------------------------

bool BVH::intersect(const vec3& origin, const vec3& dir, Hit& out) const {
    if (nodes_.empty()) return false;

    vec3 inv(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
    float best = out.t;
    int   best_tri = -1;

    int stack[64];
    int sp = 0;
    stack[sp++] = 0;

    while (sp > 0) {
        const Node& n = nodes_[stack[--sp]];
        float t_near;
        if (!aabb_hit(n.bounds, origin, inv, best, t_near)) continue;

        if (n.left < 0) { // leaf
            for (int i = n.start; i < n.start + n.count; ++i) {
                float t;
                if (tri_hit(tris_[i], origin, dir, t) && t < best) {
                    best = t;
                    best_tri = i;
                }
            }
        } else if (sp + 2 <= 64) { // guard: never write past the stack
            stack[sp++] = n.left;
            stack[sp++] = n.right;
        }
    }

    if (best_tri < 0) return false;
    out.t = best;
    out.tri = best_tri;
    return true;
}

bool BVH::occluded(const vec3& origin, const vec3& dir, float max_t) const {
    if (nodes_.empty()) return false;

    vec3 inv(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);

    int stack[64];
    int sp = 0;
    stack[sp++] = 0;

    while (sp > 0) {
        const Node& n = nodes_[stack[--sp]];
        float t_near;
        if (!aabb_hit(n.bounds, origin, inv, max_t, t_near)) continue;

        if (n.left < 0) { // leaf
            for (int i = n.start; i < n.start + n.count; ++i) {
                float t;
                if (tri_hit(tris_[i], origin, dir, t) && t < max_t)
                    return true; // any hit is enough for a shadow ray
            }
        } else if (sp + 2 <= 64) { // guard: never write past the stack
            stack[sp++] = n.left;
            stack[sp++] = n.right;
        }
    }
    return false;
}

// ---- debug -----------------------------------------------------------------

void BVH::debug_nodes(std::vector<DebugNode>& out) const {
    out.clear();
    if (nodes_.empty()) return;
    out.reserve(nodes_.size());

    // Iterative DFS carrying each node's depth.
    struct Item { int node; int depth; };
    Item stack[128];
    int sp = 0;
    stack[sp++] = { 0, 0 };

    while (sp > 0) {
        Item it = stack[--sp];
        const Node& n = nodes_[it.node];
        bool leaf = (n.left < 0);
        out.push_back({ n.bounds, it.depth, leaf });
        if (!leaf) {
            stack[sp++] = { n.left,  it.depth + 1 };
            stack[sp++] = { n.right, it.depth + 1 };
        }
    }
}

} // namespace bvh
