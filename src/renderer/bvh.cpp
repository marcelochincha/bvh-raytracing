#include <renderer/bvh.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace bvh {

namespace { thread_local long tl_node_visits = 0; }
void reset_thread_node_visits() { tl_node_visits = 0; }
long take_thread_node_visits()  { long v = tl_node_visits; tl_node_visits = 0; return v; }

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

// 30-bit Morton (Z-order) code from a normalized [0,1] point: interleaves the
// bits of x/y/z so that sorting by this code groups nearby points together.
static inline uint32_t expand_bits_10(uint32_t v) {
    v = (v | (v << 16)) & 0x030000FFu;
    v = (v | (v <<  8)) & 0x0300F00Fu;
    v = (v | (v <<  4)) & 0x030C30C3u;
    v = (v | (v <<  2)) & 0x09249249u;
    return v;
}
static inline uint32_t morton3d(float x, float y, float z) {
    auto q = [](float f) {
        return expand_bits_10((uint32_t)(std::clamp(f, 0.0f, 1.0f) * 1023.0f));
    };
    return (q(x) << 2) | (q(y) << 1) | q(z);
}

void BVH::build(std::vector<Tri> tris, BuildStrategy strategy) {
    tris_     = std::move(tris);
    strategy_ = strategy;
    nodes_.clear();
    if (tris_.empty()) return;

    centroids_.resize(tris_.size());
    for (std::size_t i = 0; i < tris_.size(); ++i)
        centroids_[i] = (tris_[i].v0 + tris_[i].v1 + tris_[i].v2) / 3.0f;

    // Morton: globally sort triangles by the Z-order code of their centroid, so
    // a plain index-median split in build_node yields a locality-ordered tree
    // (fast to build, lower traversal quality than SAH).
    if (strategy_ == Morton) {
        AABB cb = aabb_empty();
        for (const auto& c : centroids_) aabb_grow(cb, c);
        vec3 ext = cb.max - cb.min;
        std::vector<std::pair<uint32_t, int>> order(tris_.size());
        for (std::size_t i = 0; i < tris_.size(); ++i) {
            float nx = ext.x > 0 ? (centroids_[i].x - cb.min.x) / ext.x : 0.0f;
            float ny = ext.y > 0 ? (centroids_[i].y - cb.min.y) / ext.y : 0.0f;
            float nz = ext.z > 0 ? (centroids_[i].z - cb.min.z) / ext.z : 0.0f;
            order[i] = { morton3d(nx, ny, nz), (int)i };
        }
        std::sort(order.begin(), order.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        std::vector<Tri>  nt(tris_.size());
        std::vector<vec3> nc(centroids_.size());
        for (std::size_t i = 0; i < order.size(); ++i) {
            nt[i] = tris_[order[i].second];
            nc[i] = centroids_[order[i].second];
        }
        tris_       = std::move(nt);
        centroids_  = std::move(nc);
    }

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

// Median split: reorder [start, start+count) so the first half holds the
// triangles with the smallest centroid along `axis`, then return the midpoint.
// Uses quickselect, swapping tris_ and centroids_ together so they stay aligned.
int BVH::partition_median(int start, int count, int axis) {
    int mid = start + count / 2;
    int lo = start, hi = start + count - 1;
    while (lo < hi) {
        float pivot = centroids_[(lo + hi) / 2].v[axis];
        int i = lo, j = hi;
        while (i <= j) {
            while (centroids_[i].v[axis] < pivot) ++i;
            while (centroids_[j].v[axis] > pivot) --j;
            if (i <= j) {
                std::swap(tris_[i], tris_[j]);
                std::swap(centroids_[i], centroids_[j]);
                ++i; --j;
            }
        }
        if      (mid <= j) hi = j;
        else if (mid >= i) lo = i;
        else               break;
    }
    return mid;
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

    int mid;
    if (strategy_ == Morton) {
        // Already globally sorted by Z-order: a balanced index split preserves
        // spatial locality (fast build, lower-quality tree than SAH).
        mid = start + count / 2;
    } else if (strategy_ == Median) {
        // Split at the centroid median along the longest axis.
        mid = partition_median(start, count, axis);
    } else {
        // --- Binned SAH ---
        if (cext < 1e-8f) {                 // degenerate spread -> leaf
            nodes_[idx].bounds = bounds;
            nodes_[idx].left = -1;
            nodes_[idx].start = start;
            nodes_[idx].count = count;
            return idx;
        }

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

        // Compare against the cost of NOT splitting (making a leaf). The 1.0f is
        // a traversal-step constant; parent area normalizes the split cost.
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
        mid = partition(start, count, axis, cmin, scale, best_split);
        if (mid == start || mid == start + count)
            mid = start + count / 2;
    }


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
        ++tl_node_visits;                       // quality metric: nodes tested
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
        ++tl_node_visits;                       // quality metric: nodes tested
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

// ---- GPU flattening --------------------------------------------------------

void BVH::flatten(std::vector<float>& nb, std::vector<int>& nl,
                  std::vector<float>& tf) const {
    nb.resize(nodes_.size() * 8);
    nl.resize(nodes_.size() * 4);
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        const Node& n = nodes_[i];
        nb[i*8+0] = n.bounds.min.x; nb[i*8+1] = n.bounds.min.y; nb[i*8+2] = n.bounds.min.z; nb[i*8+3] = 0.0f;
        nb[i*8+4] = n.bounds.max.x; nb[i*8+5] = n.bounds.max.y; nb[i*8+6] = n.bounds.max.z; nb[i*8+7] = 0.0f;
        nl[i*4+0] = n.left; nl[i*4+1] = n.right; nl[i*4+2] = n.start; nl[i*4+3] = n.count;
    }
    tf.resize(tris_.size() * 16);
    for (std::size_t i = 0; i < tris_.size(); ++i) {
        const Tri& t = tris_[i];
        float* p = &tf[i*16];
        p[0]=t.v0.x; p[1]=t.v0.y; p[2]=t.v0.z;
        p[3]=t.v1.x; p[4]=t.v1.y; p[5]=t.v1.z;
        p[6]=t.v2.x; p[7]=t.v2.y; p[8]=t.v2.z;
        p[9]=t.normal.x; p[10]=t.normal.y; p[11]=t.normal.z;
        p[12]=t.albedo.x; p[13]=t.albedo.y; p[14]=t.albedo.z;
        p[15]=t.reflectivity;
    }
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
