#pragma once

// Optional reference backend built on Intel Embree. It builds Embree's own BVH
// over the SAME triangles our BVH uses, so the comparison study can measure our
// SAH/Median/Morton build against the industry-standard Embree kernels.
//
// Compiled only when WITH_EMBREE is defined (CMake finds embree). Otherwise the
// whole header is empty and the benchmark simply skips the Embree row, so the
// project builds everywhere.
#ifdef WITH_EMBREE

#include <embree4/rtcore.h>
#include <renderer/bvh.hpp>      // bvh::Tri, vec3
#include <vector>

namespace embree_ref {

class Scene {
public:
    Scene() {
        dev_ = rtcNewDevice(nullptr);
        sc_  = rtcNewScene(dev_);
    }
    ~Scene() {
        if (sc_)  rtcReleaseScene(sc_);
        if (dev_) rtcReleaseDevice(dev_);
    }
    Scene(const Scene&)            = delete;
    Scene& operator=(const Scene&) = delete;

    // Build an Embree triangle geometry over `tris` and commit (which builds
    // Embree's internal BVH). Vertices are stored unshared (3 per triangle) for
    // simplicity — memory is not what we compare here.
    void build(const std::vector<bvh::Tri>& tris) {
        rtcReleaseScene(sc_);
        sc_ = rtcNewScene(dev_);
        verts_.clear();
        idx_.clear();
        if (tris.empty()) { rtcCommitScene(sc_); return; }

        verts_.resize(tris.size() * 3 * 3);   // 3 verts * 3 floats per triangle
        idx_.resize(tris.size() * 3);
        for (size_t i = 0; i < tris.size(); ++i) {
            const bvh::Tri& t = tris[i];
            float* v = &verts_[i * 9];
            v[0] = t.v0.x; v[1] = t.v0.y; v[2] = t.v0.z;
            v[3] = t.v1.x; v[4] = t.v1.y; v[5] = t.v1.z;
            v[6] = t.v2.x; v[7] = t.v2.y; v[8] = t.v2.z;
            idx_[i * 3 + 0] = (unsigned)(i * 3 + 0);
            idx_[i * 3 + 1] = (unsigned)(i * 3 + 1);
            idx_[i * 3 + 2] = (unsigned)(i * 3 + 2);
        }

        RTCGeometry geom = rtcNewGeometry(dev_, RTC_GEOMETRY_TYPE_TRIANGLE);
        rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
                                   verts_.data(), 0, 3 * sizeof(float), tris.size() * 3);
        rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
                                   idx_.data(), 0, 3 * sizeof(unsigned), tris.size());
        rtcCommitGeometry(geom);
        rtcAttachGeometry(sc_, geom);
        rtcReleaseGeometry(geom);
        rtcCommitScene(sc_);   // <- builds Embree's BVH
    }

    // Nearest hit. Returns true and fills `t` on hit (no shading — this backend
    // exists to time Embree's traversal, not to render).
    bool intersect(const vec3& o, const vec3& d, float& t) const {
        RTCRayHit rh;
        rh.ray.org_x = o.x; rh.ray.org_y = o.y; rh.ray.org_z = o.z;
        rh.ray.dir_x = d.x; rh.ray.dir_y = d.y; rh.ray.dir_z = d.z;
        rh.ray.tnear = 0.0f; rh.ray.tfar = 1e30f;
        rh.ray.time  = 0.0f; rh.ray.mask = (unsigned)-1; rh.ray.flags = 0;
        rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
        RTCIntersectArguments args;
        rtcInitIntersectArguments(&args);
        rtcIntersect1(sc_, &rh, &args);
        if (rh.hit.geomID == RTC_INVALID_GEOMETRY_ID) return false;
        t = rh.ray.tfar;
        return true;
    }

private:
    RTCDevice          dev_ = nullptr;
    RTCScene           sc_  = nullptr;
    std::vector<float> verts_;   // backing storage kept alive for the geometry
    std::vector<unsigned> idx_;
};

} // namespace embree_ref
#endif // WITH_EMBREE
