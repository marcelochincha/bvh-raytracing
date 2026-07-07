#pragma once

#include <math/sr_math.hpp>
#include <renderer/sr_geometry.hpp>
#include <renderer/sr_texture.hpp>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

// =============================================================================
// Skinning rígido HORNEADO (1 hueso por vértice, sin pesos en runtime, 1 pasada
// por frame). Portado de la versión exploratoria del motor (ver O.txt).
//
// Por cada frame y hueso guardamos una "skinning matrix" final ya compuesta:
//
//     p' = M[frame][ hueso[v] ] · restP(v)
//
// restP = la malla en reposo (bind pose). Una matriz-vector por vértice.
// Coste: O(vértices) por frame. Sin jerarquía ni inverse en runtime. Como la
// geometría cambia cada frame, esto ejercita la reconstrucción del BVH dinámico
// (justo lo que queremos demostrar: skinning + BVH dinámico conviven).
//
// BINDING POR ÍNDICE (no por posición): mesh.src_vertex dice de qué 'v' del OBJ
// vino cada vértice deduplicado, así el binding (un hueso por 'v') mapea directo:
//     vertexBone[v_engine] = bone_of[ mesh.src_vertex[v_engine] ]
//
// Dos archivos de texto ('#'/vacías se saltan):
//   BINDING (weights):              ANIMACIÓN (anim):
//     bones <B>                       fps <f>
//     verts <V>                       bones <B>
//     <V enteros: hueso/vértice>      frames <F>
//                                     F*B matrices (16 floats column-major,
//                                                   orden frame -> hueso)
// El binding depende del mesh; la animación es independiente (reutilizable).
// =============================================================================
struct SkinnedMesh {
    std::vector<vertex> bind;        // pose de reposo (copia de mesh.vertices)
    std::vector<int>    vertexBone;  // hueso por vértice (paralelo a bind)
    std::vector<mat4>   skin;        // F*B matrices: skin[f*bones + b]
    int   bones   = 0;
    int   frames  = 0;
    float fps     = 24.0f;
    int   matched = 0; // vértices ligados a un hueso válido (diagnóstico)

    bool valid() const { return frames > 0 && bones > 0 && !bind.empty(); }

    // Carga binding + animación (dos archivos) y liga cada vértice de la malla a
    // su hueso por índice (vía mesh.src_vertex). `m` debe ser el OBJ que se rigeó
    // (cargado con load_obj_mesh, que rellena src_vertex). Devuelve false si algo
    // no abre, está corrupto, o el nº de huesos no coincide entre ambos archivos.
    bool load(const std::string& weights_path, const std::string& anim_path,
              const mesh& m) {
        if (m.src_vertex.size() != m.vertices.size()) return false; // no es un OBJ
        bind = m.vertices;

        // --- BINDING (weights): bones, verts, V enteros ----------------------
        std::ifstream wf(weights_path);
        if (!wf) return false;
        int bw = 0, V = 0;
        std::string line, tok;
        while (std::getline(wf, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream ss(line);
            ss >> tok;
            if      (tok == "bones") ss >> bw;
            else if (tok == "verts") { ss >> V; break; }
        }
        if (bw <= 0 || V <= 0) return false;
        std::vector<int> bone_of(V);
        for (int i = 0; i < V; ++i) wf >> bone_of[i];
        if (!wf) return false;

        // --- ANIMACIÓN (anim): fps, bones, frames, F*B matrices --------------
        std::ifstream af(anim_path);
        if (!af) return false;
        while (std::getline(af, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream ss(line);
            ss >> tok;
            if      (tok == "fps")    ss >> fps;
            else if (tok == "bones")  ss >> bones;
            else if (tok == "frames") { ss >> frames; break; }
        }
        if (bones <= 0 || frames <= 0 || bones != bw) return false; // huesos != binding
        skin.resize((std::size_t)frames * bones);
        for (auto& M : skin)
            for (int i = 0; i < 16; ++i) af >> M.m[i];
        if (!af) return false;

        // El binding debe cubrir cada 'v' del OBJ.
        uint32_t maxSrc = 0;
        for (uint32_t s : m.src_vertex) if (s > maxSrc) maxSrc = s;
        if ((int)maxSrc >= V) return false; // nº de 'v' del OBJ != verts del binding

        vertexBone.resize(bind.size());
        matched = 0;
        for (std::size_t v = 0; v < bind.size(); ++v) {
            vertexBone[v] = bone_of[m.src_vertex[v]];
            ++matched;
        }
        return true;
    }

    // Escribe en la malla la pose en el tiempo `t` (SEGUNDOS, real-time). El fps
    // del bake solo fija la duración (frames/fps); el motor muestrea por tiempo e
    // INTERPOLA entre los dos frames vecinos, así el resultado es independiente
    // de si el clip se horneó a 30 o 60 fps. Se clampa al último frame (no hace
    // loop: el timing/looping lo decide quien llama). Marca la malla dirty para
    // que el BVH dinámico se reconstruya.
    //
    // La interpolación es lineal por componente de la matriz de skinning. No es
    // exacta (no re-ortonormaliza la rotación), pero con el paso entre frames de
    // un bake es visualmente correcta y barata: una sola pasada por hueso.
    void apply(mesh& m, float t) const {
        if (!valid()) return;
        float ff = t * fps;
        if (ff < 0.0f) ff = 0.0f;
        float maxf = (float)(frames - 1);
        if (ff > maxf) ff = maxf;
        int   f0 = (int)ff;
        int   f1 = (f0 + 1 < frames) ? f0 + 1 : f0;
        float a  = ff - (float)f0;

        // Matriz interpolada por hueso (bones pequeño; se reutiliza por vértice).
        std::vector<mat4> Mb((std::size_t)bones);
        const mat4* A = &skin[(std::size_t)f0 * bones];
        const mat4* B = &skin[(std::size_t)f1 * bones];
        for (int b = 0; b < bones; ++b)
            for (int i = 0; i < 16; ++i)
                Mb[b].m[i] = A[b].m[i] * (1.0f - a) + B[b].m[i] * a;

        for (std::size_t v = 0; v < bind.size(); ++v) {
            vec4 p = Mb[vertexBone[v]] * bind[v].p;
            m.vertices[v].p = vec3(p.x, p.y, p.z);
        }
        m._modelMatrixDirty = true;
    }
};

// Diffuse-texture cache shared by all OBJ meshes loaded here (owned for the whole
// program, like the engine's other leaked assets). Returns null if it can't load.
inline const texture* obj_cached_texture(const std::string& path) {
    static std::map<std::string, texture*> cache;
    auto it = cache.find(path);
    if (it != cache.end()) return it->second;
    texture* t = new texture();
    if (!load_png_texture(path, *t)) { delete t; cache[path] = nullptr; return nullptr; }
    cache[path] = t;
    return t;
}

// -----------------------------------------------------------------------------
// Cargador OBJ -> mesh (vertices + faces + src_vertex + textura difusa).
// Deduplica por par (v,t) como haría un exportador, recuerda de qué 'v' del OBJ
// vino cada vértice para que SkinnedMesh::load pueda ligar el binding por índice,
// y resuelve mtllib->map_Kd para rellenar out.tex (primer material texturizado;
// el binding rígido usa una sola malla/textura). Devuelve false si no abre.
// -----------------------------------------------------------------------------
inline bool load_obj_mesh(const std::string& path, mesh& out, float scale = 1.0f) {
    std::ifstream f(path);
    if (!f) return false;

    std::string dir(path);
    auto sl = dir.find_last_of("/\\");
    dir = (sl != std::string::npos) ? dir.substr(0, sl + 1) : "";

    std::vector<vec3> verts;
    std::vector<vec2> uvs;
    std::map<std::pair<int,int>, uint32_t> dedup; // (vi,ti) -> vertex index
    out.vertices.clear();
    out.faces.clear();
    out.src_vertex.clear();
    out.tex = nullptr;

    // Reads the first map_Kd from an .mtl and loads it into out.tex.
    auto load_mtl = [&](const std::string& fname) {
        std::ifstream mf(dir + fname);
        if (!mf) return;
        std::string line;
        while (std::getline(mf, line)) {
            std::istringstream ss(line);
            std::string key; ss >> key;
            if (key == "map_Kd") {
                std::string fn; std::getline(ss, fn);
                size_t a = fn.find_first_not_of(" \t");
                if (a != std::string::npos && !out.tex)
                    out.tex = obj_cached_texture(dir + fn.substr(a));
            }
        }
    };

    auto emit = [&](int vi, int ti) -> uint32_t {
        auto key = std::make_pair(vi, ti);
        auto it = dedup.find(key);
        if (it != dedup.end()) return it->second;
        uint32_t idx = (uint32_t)out.vertices.size();
        vertex ve;
        ve.p = (vi >= 0 && vi < (int)verts.size()) ? verts[vi] * scale : vec3(0, 0, 0);
        ve.t = (ti >= 0 && ti < (int)uvs.size())    ? uvs[ti]           : vec2(0, 0);
        out.vertices.push_back(ve);
        out.src_vertex.push_back((uint32_t)vi);
        dedup[key] = idx;
        return idx;
    };

    std::string line;
    while (std::getline(f, line)) {
        const char* s = line.c_str();
        while (*s == ' ' || *s == '\t') ++s;
        if (s[0] == 'v' && (s[1] == ' ' || s[1] == '\t')) {
            char* e; const char* p = s + 2;
            float x = std::strtof(p, &e); p = e;
            float y = std::strtof(p, &e); p = e;
            float z = std::strtof(p, &e);
            verts.push_back(vec3(x, y, z));
        } else if (s[0] == 'v' && s[1] == 't' && (s[2] == ' ' || s[2] == '\t')) {
            char* e; const char* p = s + 3;
            float u = std::strtof(p, &e); p = e;
            float v = std::strtof(p, &e);
            uvs.push_back(vec2(u, v));
        } else if (s[0] == 'f' && (s[1] == ' ' || s[1] == '\t')) {
            const int nv = (int)verts.size(), nt = (int)uvs.size();
            std::vector<uint32_t> poly;
            const char* p = s + 2;
            while (*p) {
                while (*p == ' ' || *p == '\t') ++p;
                if (!*p) break;
                char* e;
                long vi = std::strtol(p, &e, 10); p = e;
                long ti = 0; bool hasT = false;
                if (*p == '/') {
                    ++p;
                    if (*p != '/') { ti = std::strtol(p, &e, 10); p = e; hasT = true; }
                    if (*p == '/') { ++p; std::strtol(p, &e, 10); p = e; } // skip vn
                }
                if (vi == 0) continue;
                int vIdx = (vi > 0) ? (int)vi - 1 : nv + (int)vi;
                int tIdx = (hasT && ti != 0) ? ((ti > 0) ? (int)ti - 1 : nt + (int)ti) : -1;
                poly.push_back(emit(vIdx, tIdx));
            }
            for (std::size_t i = 1; i + 1 < poly.size(); ++i)
                out.faces.push_back({ poly[0], poly[i], poly[i + 1] });
        } else if (s[0] == 'm') { // mtllib
            std::istringstream ss(s);
            std::string tag, fn; ss >> tag >> fn;
            if (tag == "mtllib") load_mtl(fn);
        }
    }
    out._modelMatrixDirty = true;
    return true;
}

// -----------------------------------------------------------------------------
// Generador procedural: un "tentáculo"/personaje-columna hecho de anillos
// apilados y ligado a una cadena de huesos, con una animación de onda que lo
// dobla. No necesita assets: sirve para probar de una que el skinning deforma
// geometría por vértice cada frame y que el BVH dinámico lo soporta.
//
// La skinning matrix se hornea directo (sin inverse general): como el bind de
// cada hueso b es una traslación pura a P_b = (0, b*seg, 0), su inversa es
// translate(-P_b), así que  S_b = A_b · translate(-P_b), con A_b el transform
// animado acumulado de la cadena.
// -----------------------------------------------------------------------------
inline void build_procedural_character(mesh& out, SkinnedMesh& skin) {
    const int   bones      = 7;      // articulaciones de la cadena
    const float seg        = 0.34f;  // largo de cada segmento (altura)
    const int   ringsPerSeg = 4;
    const int   slices     = 14;     // vértices por anillo
    const int   rings      = bones * ringsPerSeg;         // anillos a lo alto
    const float rBase      = 0.30f, rTip = 0.10f;
    const float pi         = 3.14159265358979f;

    out.vertices.clear();
    out.faces.clear();
    out.src_vertex.clear();
    skin.vertexBone.clear();

    // --- geometría en reposo (columna recta a lo largo de +Y) ----------------
    for (int r = 0; r <= rings; ++r) {
        float ft = (float)r / rings;      // 0..1 a lo alto
        float h  = ft * (bones * seg);
        float rad = rBase + (rTip - rBase) * ft;
        int bone = (int)std::lround(h / seg);
        if (bone < 0) bone = 0; if (bone >= bones) bone = bones - 1;
        for (int sdiv = 0; sdiv < slices; ++sdiv) {
            float a = (float)sdiv / slices * 2.0f * pi;
            vertex ve;
            ve.p = vec3(std::cos(a) * rad, h, std::sin(a) * rad);
            ve.t = vec2((float)sdiv / slices, ft);
            out.vertices.push_back(ve);
            out.src_vertex.push_back((uint32_t)out.vertices.size() - 1);
            skin.vertexBone.push_back(bone);
        }
    }
    // tapa superior (un vértice central ligado al último hueso)
    uint32_t capIdx = (uint32_t)out.vertices.size();
    {
        vertex ve; ve.p = vec3(0.0f, bones * seg, 0.0f); ve.t = vec2(0.5f, 1.0f);
        out.vertices.push_back(ve);
        out.src_vertex.push_back(capIdx);
        skin.vertexBone.push_back(bones - 1);
    }

    auto ring = [&](int r, int s) -> uint32_t {
        return (uint32_t)(r * slices + (s % slices));
    };
    for (int r = 0; r < rings; ++r)
        for (int s = 0; s < slices; ++s) {
            uint32_t a = ring(r, s),     b = ring(r, s + 1);
            uint32_t c = ring(r + 1, s), d = ring(r + 1, s + 1);
            out.faces.push_back({ a, c, b });
            out.faces.push_back({ b, c, d });
        }
    for (int s = 0; s < slices; ++s) // tapa
        out.faces.push_back({ ring(rings, s), capIdx, ring(rings, s + 1) });

    // --- horneado de la animación (onda viajera que dobla la cadena) ---------
    const int   frames = 60;   // 2.5 s de loop a 24 fps
    skin.bind    = out.vertices;
    skin.bones   = bones;
    skin.frames  = frames;
    skin.fps     = 24.0f;
    skin.matched = (int)out.vertices.size();
    skin.skin.assign((std::size_t)frames * bones, mat4(1.0f));

    const vec3 zAxis(0.0f, 0.0f, 1.0f);
    for (int f = 0; f < frames; ++f) {
        float ph = (float)f / frames * 2.0f * pi;
        mat4 A(1.0f);                 // transform animado acumulado
        vec3 prevP(0.0f, 0.0f, 0.0f); // P_{b-1}
        for (int b = 0; b < bones; ++b) {
            vec3 P(0.0f, b * seg, 0.0f);
            // ángulo por junta: onda que viaja hacia la punta, base más quieta
            float amp   = 0.22f + 0.03f * b;
            float theta = amp * std::sin(ph - b * 0.9f);
            A = A * translationMatrix(P - prevP) * rotationMatrix(theta, zAxis);
            // S_b = A_b * inverse(bind_b) = A_b * translate(-P_b)
            skin.skin[(std::size_t)f * bones + b] = A * translationMatrix(vec3(0,0,0) - P);
            prevP = P;
        }
    }
    out._modelMatrixDirty = true;
}
