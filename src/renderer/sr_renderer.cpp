#include <renderer/sr_renderer.hpp>
#include <cmath>
#include <algorithm>

// Fog parameters (simple depth-based fade in eye space meters)
static constexpr float kFogStart = 5.0f;
static constexpr float kFogEnd = 100.0f;

// Light bluish
static constexpr uint32_t kFogColor = 0xFFADD8E6;
// Global guard-band factor for clip planes; set per draw call if needed.
static float g_clipGuard = 1.0f + 1e-3f;
typedef float (*insidePlaneFn)(const clipVertex &);
struct clipBuffer
{
    clipVertex buf[8];
    int count = 0;

    void clear()
    {
        count = 0;
    }

    inline void add(const clipVertex &v)
    {
        if (count < 8)
        {
            buf[count++] = v;
        }
    }
};
// static clipBuffer s_clipBuffer;

inline void sort_vertices_by_y(vec3 &v0, vec3 &v1, vec3 &v2)
{
    if (v1.y < v0.y)
        std::swap(v0, v1);
    if (v2.y < v0.y)
        std::swap(v0, v2);
    if (v2.y < v1.y)
        std::swap(v1, v2);
}

inline void sort_raster_coord_by_y(rasterCoord &v0, rasterCoord &v1, rasterCoord &v2)
{
    if (v1.p.y < v0.p.y)
        std::swap(v0, v1);
    if (v2.p.y < v0.p.y)
        std::swap(v0, v2);
    if (v2.p.y < v1.p.y)
        std::swap(v1, v2);
}

inline float interpolate_by_y(vec3 a, vec3 b, float y, vec_component c)
{
    if (a.y == b.y)
        return a.v[c];
    return a.v[c] + (b.v[c] - a.v[c]) * ((y - a.y) / (b.y - a.y));
}

inline vec3 convert_to_fb(const framebuffer &fb, const vec3 &v)
{
    return vec3(
        (v.x + 1.0f) * 0.5f * (fb.width - 1),
        (-v.y + 1.0f) * 0.5f * (fb.height - 1),
        v.z);
}

uint32_t brightness_color(uint32_t color, float factor)
{
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    r = static_cast<uint8_t>(std::min(255.0f, r * factor));
    g = static_cast<uint8_t>(std::min(255.0f, g * factor));
    b = static_cast<uint8_t>(std::min(255.0f, b * factor));

    return (r << 16) | (g << 8) | b;
}

static inline uint32_t lerp_color(uint32_t a, uint32_t b, float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    uint8_t ar = (a >> 16) & 0xFF, ag = (a >> 8) & 0xFF, ab = a & 0xFF;
    uint8_t br = (b >> 16) & 0xFF, bg = (b >> 8) & 0xFF, bb = b & 0xFF;
    uint8_t r = static_cast<uint8_t>(ar + (br - ar) * t);
    uint8_t g = static_cast<uint8_t>(ag + (bg - ag) * t);
    uint8_t bch = static_cast<uint8_t>(ab + (bb - ab) * t);
    return (r << 16) | (g << 8) | bch;
}

uint32_t sample_texture(const texture *tex, float s, float t)
{
    if (tex == nullptr || tex->data == nullptr)
        return 0xFFFFFFFF; // White color if no texture

    int tx = int(s * float(tex->width));
    int ty = int(t * float(tex->height));
    tx = std::max(0, std::min(tex->width - 1, tx));
    ty = std::max(0, std::min(tex->height - 1, ty));
    return tex->data[ty * tex->width + tx];
}

vec3 get_barycentric_coords(float x, float y, const vec3 &v0, const vec3 &v1, const vec3 &v2)
{
    float x1 = v0.x, y1 = v0.y;
    float x2 = v1.x, y2 = v1.y;
    float x3 = v2.x, y3 = v2.y;
    float denom = ((y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3));

    if (std::abs(denom) < 1e-6f)
    {
        return vec3(-1, -1, -1);
    }

    vec3 r;
    r.x = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / denom;
    r.y = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / denom;
    r.z = 1.0f - r.x - r.y;
    return r;
}

vec3 get_normal(const vec3 &v0, const vec3 &v1, const vec3 &v2)
{
    vec3 vA = v1 - v0;
    vec3 vB = v2 - v0;
    return normalize(cross(vA, vB));
}

clipVertex lerp_clip(const clipVertex &a, const clipVertex &b, float t)
{
    clipVertex r;
    r.pos = a.pos + (b.pos - a.pos) * t;
    r.uv = a.uv + (b.uv - a.uv) * t;
    return r;
}

// Usamos punteros o referencias a los buffers estáticos que definimos antes
void clip_against_plane(const clipBuffer &in, clipBuffer &out, insidePlaneFn inside_fn)
{
    out.count = 0;
    if (in.count == 0)
        return;

    constexpr float eps = 1e-6f;

    // S es el último vértice del polígono de entrada
    clipVertex S = in.buf[in.count - 1];
    float dS = inside_fn(S);

    for (int i = 0; i < in.count; ++i)
    {
        const clipVertex &E = in.buf[i];
        float dE = inside_fn(E);

        bool S_in = dS <= eps;
        bool E_in = dE <= eps;

        if (S_in && E_in)
        {
            out.add(E);
        }
        else if (S_in && !E_in)
        {
            float denom = dS - dE;
            if (std::abs(denom) > eps)
            {
                out.add(lerp_clip(S, E, dS / denom));
            }
        }
        else if (!S_in && E_in)
        {
            float denom = dS - dE;
            if (std::abs(denom) > eps)
            {
                out.add(lerp_clip(S, E, dS / denom));
            }
            out.add(E);
        }
        S = E;
        dS = dE;
    }
}

float plane_left(const clipVertex &v) { return v.pos.x - v.pos.w * g_clipGuard; }
float plane_right(const clipVertex &v) { return -v.pos.x - v.pos.w * g_clipGuard; }
float plane_top(const clipVertex &v) { return v.pos.y - v.pos.w * g_clipGuard; }
float plane_bottom(const clipVertex &v) { return -v.pos.y - v.pos.w * g_clipGuard; }
float plane_far(const clipVertex &v) { return v.pos.z - v.pos.w * g_clipGuard; }
float plane_near(const clipVertex &v) { return -v.pos.z - v.pos.w * g_clipGuard; }

void draw_line(framebuffer &fb, int x0, int y0, int x1, int y1, uint32_t color, bool use_depth)
{
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        if (x0 >= 0 && x0 < fb.width && y0 >= 0 && y0 < fb.height)
        {
            fb.colorBuffer[y0 * fb.width + x0] = color;
            if (use_depth)
            {
                fb.depthBuffer[y0 * fb.width + x0] = 0.0f;
            }
        }
        if (x0 == x1 && y0 == y1)
            break;
        int err2 = 2 * err;
        if (err2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }
        if (err2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

void render_triangle(framebuffer &fb, rasterCoord cv0, rasterCoord cv1, rasterCoord cv2, const renderConfig &config, float lightFactor, bool findZinf)
{
    sort_raster_coord_by_y(cv0, cv1, cv2);
    int y_start = std::ceil(cv0.p.y);
    int y_end = std::floor(cv2.p.y);

    vec3 &v0 = cv0.p;
    vec3 &v1 = cv1.p;
    vec3 &v2 = cv2.p;

    vec3 &v0_uv = cv0.t;
    vec3 &v1_uv = cv1.t;
    vec3 &v2_uv = cv2.t;

    float x1, x2;
    float z1, z2;
    float min_z = std::min({v0.z, v1.z, v2.z});
    float max_z = std::max({v0.z, v1.z, v2.z});

    for (int y = y_start; y <= y_end; ++y)
    {
        x1 = interpolate_by_y(v0, v2, float(y), DIM_X);
        z1 = interpolate_by_y(v0, v2, float(y), DIM_Z);
        if (y < v1.y)
        {
            x2 = interpolate_by_y(v0, v1, float(y), DIM_X);
            z2 = interpolate_by_y(v0, v1, float(y), DIM_Z);
        }
        else
        {
            x2 = interpolate_by_y(v1, v2, float(y), DIM_X);
            z2 = interpolate_by_y(v1, v2, float(y), DIM_Z);
        }

        if (x1 > x2)
        {
            std::swap(x1, x2);
            std::swap(z1, z2);
        }

        int x_start = std::ceil(x1);
        int x_end = std::floor(x2);

        for (int x = x_start; x <= x_end; ++x)
        {
            float z_denom = (x2 - x1);
            if (std::abs(z_denom) < 1e-6f)
            {
                continue;
            }
            float z = z1 + (z2 - z1) * ((float(x) - x1) / z_denom);
            float depth_z = fb.depthBuffer[y * fb.width + x];
            if (x >= 0 && x < fb.width && y >= 0 && y < fb.height && ((z < depth_z  && !findZinf) || (findZinf && depth_z == 1.0f)))
            {
                vec3 bar = get_barycentric_coords(float(x), float(y), v0, v1, v2);
                float z_inv = bar.x * v0_uv.z + bar.y * v1_uv.z + bar.z * v2_uv.z;

                uint32_t bColor = 0x0;
                if (config.tex == nullptr)
                    bColor = config.baseColor;
                else
                {
                    // For now texture sampling with barycentric coords without light
                    float uz = bar.x * v0_uv.x + bar.y * v1_uv.x + bar.z * v2_uv.x;
                    float vz = bar.x * v0_uv.y + bar.y * v1_uv.y + bar.z * v2_uv.y;
                    float s = uz / z_inv;
                    float t = vz / z_inv;
                    bColor = sample_texture(config.tex, s, t);
                }

                float realFactor = 0.5f + lightFactor * 0.5f;

                // Eye-space depth from interpolated 1/w (z_inv)
                float eyeDepth = (z_inv > 1e-6f) ? (1.0f / z_inv) : kFogEnd;
                // Fog factor based on eye-space distance
                float fogT = (eyeDepth - kFogStart) / (kFogEnd - kFogStart);
                fogT = 0;//std::clamp(fogT, 0.0f, 1.0f);

                uint32_t lit = brightness_color(bColor, realFactor);
                uint32_t finalColor = lerp_color(lit, kFogColor, fogT); // fogT=0 -> lit, fogT=1 -> fog

                fb.colorBuffer[y * fb.width + x] = finalColor;
                if (!config.ignoreDepth)
                    fb.depthBuffer[y * fb.width + x] = z;
            }
        }
    }
}

float check_if_visible(vec3 screen_v0, vec3 screen_v1, vec3 screen_v2)
{
    vec2 e0 = screen_v1 - screen_v0;
    vec2 e1 = screen_v2 - screen_v0;

    float area = e0.x * e1.y - e0.y * e1.x;
    return area;
}

// Render mesh using config, can be modified to use different configs
void render_mesh(framebuffer &fb, const camera &cam, const mesh &m, renderConfig &config)
{
    // Static clip buffers to avoid reallocating every draw call, change if multithreading
    static clipBuffer A, B;

    mat4 mvp = cam.projection() * cam.view() * m.modelMatrix();
    mat4 mv = cam.view() * m.modelMatrix();
    for (int index = 0; index < static_cast<int>(m.faces.size()); ++index)
    {
        const triangle &tri = m.faces[index];

        clipVertex v0{mvp * m.vertices[tri.v0].p, m.vertices[tri.v0].t};
        clipVertex v1{mvp * m.vertices[tri.v1].p, m.vertices[tri.v1].t};
        clipVertex v2{mvp * m.vertices[tri.v2].p, m.vertices[tri.v2].t};

        // Compute light
        vec3 l_v0 = mv * vec4(m.vertices[tri.v0].p.x, m.vertices[tri.v0].p.y, m.vertices[tri.v0].p.z, 1.0f);
        vec3 l_v1 = mv * vec4(m.vertices[tri.v1].p.x, m.vertices[tri.v1].p.y, m.vertices[tri.v1].p.z, 1.0f);
        vec3 l_v2 = mv * vec4(m.vertices[tri.v2].p.x, m.vertices[tri.v2].p.y, m.vertices[tri.v2].p.z, 1.0f);

        // Not get normal
        vec3 normal = normalize(get_normal(l_v0, l_v1, l_v2));
        if (m.inverseFaces)
            normal = -normal;
        // Compute cross
        float d = dot(normal, vec3(0.0f, 0.0f, -1.0f));
        float lightFactor;
        if (config.ignoreLight)
            lightFactor = 1.0f;
        else
            lightFactor = std::max(0.0f, d * config.lightInfluence);

        // if (check_if_visible(base_screen_v0, base_screen_v1, base_screen_v2) > 0)
        //     continue;

        // initiate poly
        A.clear();
        A.add(v0);
        A.add(v1);
        A.add(v2);

        // clip against planes
        clip_against_plane(A, B, plane_left);
        A.clear();
        clip_against_plane(B, A, plane_right);
        B.clear();
        clip_against_plane(A, B, plane_top);
        A.clear();
        clip_against_plane(B, A, plane_bottom);
        B.clear();
        clip_against_plane(A, B, plane_near);
        A.clear();
        clip_against_plane(B, A, plane_far);

        // Result is A
        const clipBuffer &poly = A;
        clipVertex base = poly.buf[0];
        for (size_t i = 1; i + 1 < poly.count; ++i)
        {
            clipVertex a = poly.buf[i];
            clipVertex b = poly.buf[i + 1];

            vec3 ndc_v0 = base.pos / base.pos.w;
            vec3 ndc_v1 = a.pos / a.pos.w;
            vec3 ndc_v2 = b.pos / b.pos.w;

            vec3 screen_v0 = convert_to_fb(fb, ndc_v0);
            vec3 screen_v1 = convert_to_fb(fb, ndc_v1);
            vec3 screen_v2 = convert_to_fb(fb, ndc_v2);

            vec2 e0 = screen_v1 - screen_v0;
            vec2 e1 = screen_v2 - screen_v0;

            float area = e0.x * e1.y - e0.y * e1.x;
            if (m.inverseFaces)
                area = -area;
            if (area < 0)
                continue;

            vec3 uvw0 = vec3(base.uv.x, base.uv.y, 1.0f) / base.pos.w;
            vec3 uvw1 = vec3(a.uv.x, a.uv.y, 1.0f) / a.pos.w;
            vec3 uvw2 = vec3(b.uv.x, b.uv.y, 1.0f) / b.pos.w;

            rasterCoord cv0 = {screen_v0, uvw0};
            rasterCoord cv1 = {screen_v1, uvw1};
            rasterCoord cv2 = {screen_v2, uvw2};

            render_triangle(fb, cv0, cv1, cv2, config, lightFactor, false);
        }
    }
}

//Draw skybox

void render_skybox(framebuffer &fb, const camera &cam, std::array<texture, 6> &skyboxTextures)
{
    static clipBuffer A, B;
    static renderConfig config{
        .baseColor = 0xFFFFFFFF,
        .tex = nullptr,
        .lightInfluence = 0.0f,
        .ignoreDepth = true,
        .ignoreLight = true,
    };
    const static mesh skyboxMesh = create_skybox_mesh();
    // We can optimize by precomputing this MVP since the skybox doesn't move, but for simplicity we compute it every frame
    mat4 mvp = cam.projection() * transpose(cam.rotation()); // TRANSPOSE because we want to ignore rotation for the skybox, only use projection.
    for (int index = 0; index < static_cast<int>(skyboxMesh.faces.size()); ++index)
    {
        const triangle &tri = skyboxMesh.faces[index];

        clipVertex v0{mvp * skyboxMesh.vertices[tri.v0].p, skyboxMesh.vertices[tri.v0].t};
        clipVertex v1{mvp * skyboxMesh.vertices[tri.v1].p, skyboxMesh.vertices[tri.v1].t};
        clipVertex v2{mvp * skyboxMesh.vertices[tri.v2].p, skyboxMesh.vertices[tri.v2].t};

        // initiate poly
        A.clear();
        A.add(v0);
        A.add(v1);
        A.add(v2);

        // clip against planes
        clip_against_plane(A, B, plane_left);
        A.clear();
        clip_against_plane(B, A, plane_right);
        B.clear();
        clip_against_plane(A, B, plane_top);
        A.clear();
        clip_against_plane(B, A, plane_bottom);
        B.clear();
        clip_against_plane(A, B, plane_near);
        A.clear();
        clip_against_plane(B, A, plane_far);

        // Result is A
        const clipBuffer &poly = A;
        clipVertex base = poly.buf[0];
        for (size_t i = 1; i + 1 < poly.count; ++i)
        {
            clipVertex a = poly.buf[i];
            clipVertex b = poly.buf[i + 1];

            vec3 ndc_v0 = base.pos / base.pos.w;
            vec3 ndc_v1 = a.pos / a.pos.w;
            vec3 ndc_v2 = b.pos / b.pos.w;

            vec3 screen_v0 = convert_to_fb(fb, ndc_v0);
            vec3 screen_v1 = convert_to_fb(fb, ndc_v1);
            vec3 screen_v2 = convert_to_fb(fb, ndc_v2);

            vec3 uvw0 = vec3(base.uv.x, base.uv.y, 1.0f) / base.pos.w;
            vec3 uvw1 = vec3(a.uv.x, a.uv.y, 1.0f) / a.pos.w;
            vec3 uvw2 = vec3(b.uv.x, b.uv.y, 1.0f) / b.pos.w;
            rasterCoord cv0 = {screen_v0, uvw0};
            rasterCoord cv1 = {screen_v1, uvw1};
            rasterCoord cv2 = {screen_v2, uvw2};
            config.tex = &skyboxTextures[index / 2]; // Each face of the skybox corresponds to 2 triangles, so we divide by 2 to get the correct texture index
            render_triangle(fb, cv0, cv1, cv2, config, 1.0f, true);
        }
    }
}

void draw_gizmo_line(framebuffer &fb, const camera &cam, const vec3 &start, const vec3 &end, uint32_t color)
{
    vec4 start_clip = cam.projection() * cam.view() * vec4(start.x, start.y, start.z, 1.0f);
    vec4 end_clip = cam.projection() * cam.view() * vec4(end.x, end.y, end.z, 1.0f);

    if (start_clip.w <= 0 || end_clip.w <= 0)
        return;

    vec3 start_ndc = start_clip / start_clip.w;
    vec3 end_ndc = end_clip / end_clip.w;

    vec3 start_screen = convert_to_fb(fb, start_ndc);
    vec3 end_screen = convert_to_fb(fb, end_ndc);
    if (start_screen.x < 0 || end_screen.x < 0 || start_screen.x >= fb.width || end_screen.x >= fb.width ||
        start_screen.y < 0 || end_screen.y < 0 || start_screen.y >= fb.height || end_screen.y >= fb.height)
        return;

    draw_line(fb, static_cast<int>(start_screen.x), static_cast<int>(start_screen.y), static_cast<int>(end_screen.x), static_cast<int>(end_screen.y), color);
}

void render_gizmo(framebuffer &fb, const camera &cam, const vec3 &pos, float size)
{
    draw_gizmo_line(fb, cam, pos, pos + vec3(size, 0, 0), 0xFFFF0000);
    draw_gizmo_line(fb, cam, pos, pos + vec3(0, size, 0), 0xFF00FF00);
    draw_gizmo_line(fb, cam, pos, pos + vec3(0, 0, size), 0xFF0000FF);
}
