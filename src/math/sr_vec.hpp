#pragma once

#include <cmath>
/*
    VECTORS: Simple vector structure with basic operations. (2D, 3D, 4D)
*/

enum vec_component
{
    DIM_X = 0,
    DIM_Y = 1,
    DIM_Z = 2
};


union vec4;

union vec2
{
    struct
    {
        float x, y;
    };
    float v[2];

    vec2() : x(0), y(0) {}
    vec2(float x, float y) : x(x), y(y) {}
};

union vec3
{
    struct
    {
        float x, y, z;
    };
    float v[3];

    vec3() : x(0), y(0), z(0) {}
    vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    vec3(const vec4& v);
};

union vec4
{
    struct
    {
        float x, y, z, w;
    };
    float v[4];

    vec4() : x(0), y(0), z(0), w(0) {}
    vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
};

// ---------- vec2 ----------
inline vec2 operator+(vec2 a, vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline vec2 operator-(vec2 a, vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline vec2 operator*(vec2 a, float s) { return {a.x * s, a.y * s}; }
inline vec2 operator/(vec2 a, float s) { return {a.x / s, a.y / s}; }

inline float dot(vec2 a, vec2 b) { return a.x * b.x + a.y * b.y; }
inline float cross(vec2 a, vec2 b) { return a.x * b.y - a.y * b.x; }
inline float length(vec2 a) { return std::sqrt(dot(a, a)); }
inline vec2 normalize(vec2 a)
{
    float l = length(a);
    return l ? a / l : a;
}

// ---------- vec3 ----------
inline vec3 operator+(vec3 a, vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline vec3 operator-(vec3 a, vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline vec3 operator*(vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline vec3 operator/(vec3 a, float s) { return {a.x / s, a.y / s, a.z / s}; }

inline float dot(vec3 a, vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline vec3 cross(vec3 a, vec3 b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x};
}
inline float length(vec3 a) { return sqrtf(dot(a, a)); }
inline vec3 normalize(vec3 a)
{
    float l = length(a);
    return l ? a / l : a;
}

inline vec3::vec3(const vec4 &v) : x(v.x), y(v.y), z(v.z) {}


// ---------- vec4 ----------
inline vec4 operator+(vec4 a, vec4 b) { return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w}; }
inline vec4 operator-(vec4 a, vec4 b) { return {a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w}; }
inline vec4 operator*(vec4 a, float s) { return {a.x * s, a.y * s, a.z * s, a.w * s}; }
inline vec4 operator/(vec4 a, float s) { return {a.x / s, a.y / s, a.z / s, a.w / s}; }

inline float dot(vec4 a, vec4 b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
inline float length(vec4 a) { return sqrtf(dot(a, a)); }
inline vec4 normalize(vec4 a)
{
    float l = length(a);
    return l ? a / l : a;
}

// Include a lerp function for vec2, vec3, vec4
inline vec2 lerp(vec2 a, vec2 b, float t) { return a * (1 - t) + b * t; }
inline vec3 lerp(vec3 a, vec3 b, float t) { return a * (1 - t) + b * t; }
inline vec4 lerp(vec4 a, vec4 b, float t) { return a * (1 - t) + b * t; }