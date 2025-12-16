#pragma once

#include <cmath>
#include "sr_vec.hpp"

//Column -major 4x4 matrix
struct mat4
{
    float m[16] = {};

    mat4(){}

    mat4(float diagonal)
    {
        for (int i = 0; i < 16; ++i)
            m[i] = (i % 5 == 0) ? diagonal : 0.0f;
        //m[15] = 1.0f; // Homogeneous coordinate
    }

    // Glm style constructor for convenience
    mat4(
        float c0rX, float c0rY, float c0rZ, float c0rW,
        float c1rX, float c1rY, float c1rZ, float c1rW,
        float c2rX, float c2rY, float c2rZ, float c2rW,
        float c3rX, float c3rY, float c3rZ, float c3rW)
    {
        m[0] = c0rX;
        m[1] = c0rY;
        m[2] = c0rZ;
        m[3] = c0rW;
        m[4] = c1rX;
        m[5] = c1rY;
        m[6] = c1rZ;
        m[7] = c1rW;
        m[8] = c2rX;
        m[9] = c2rY;
        m[10] = c2rZ;
        m[11] = c2rW;
        m[12] = c3rX;
        m[13] = c3rY;
        m[14] = c3rZ;
        m[15] = c3rW;
    }

    mat4(const vec4 &col0, const vec4 &col1, const vec4 &col2, const vec4 &col3)
    {
        m[0] = col0.x;
        m[1] = col0.y;
        m[2] = col0.z;
        m[3] = col0.w;
        m[4] = col1.x;
        m[5] = col1.y;
        m[6] = col1.z;
        m[7] = col1.w;
        m[8] = col2.x;
        m[9] = col2.y;
        m[10] = col2.z;
        m[11] = col2.w;
        m[12] = col3.x;
        m[13] = col3.y;
        m[14] = col3.z;
        m[15] = col3.w;
    }

    // Access operator
    inline float &operator()(int row, int col)
    {
        return m[col * 4 + row];
    }

    inline const float &operator()(int row, int col) const
    {
        return m[col * 4 + row];
    }
};

// mat4 operators
inline mat4 operator*(const mat4 &a, const mat4 &b)
{
    mat4 result;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            result(i, j) = a(i, 0) * b(0, j) +
                           a(i, 1) * b(1, j) +
                           a(i, 2) * b(2, j) +
                           a(i, 3) * b(3, j);
        }
    }
    return result;
}   

inline vec4 operator*(const mat4 &m, const vec3 &v)
{
    return vec4(
        v.x * m.m[0] + v.y * m.m[4] + v.z * m.m[8] + m.m[12],
        v.x * m.m[1] + v.y * m.m[5] + v.z * m.m[9] + m.m[13],
        v.x * m.m[2] + v.y * m.m[6] + v.z * m.m[10] + m.m[14],
        v.x * m.m[3] + v.y * m.m[7] + v.z * m.m[11] + m.m[15]);
}

inline vec4 operator*(const mat4 &m, const vec4 &v)
{
    return vec4(
        v.x * m.m[0] + v.y * m.m[4] + v.z * m.m[8] + v.w * m.m[12],
        v.x * m.m[1] + v.y * m.m[5] + v.z * m.m[9] + v.w * m.m[13],
        v.x * m.m[2] + v.y * m.m[6] + v.z * m.m[10] + v.w * m.m[14],
        v.x * m.m[3] + v.y * m.m[7] + v.z * m.m[11] + v.w * m.m[15]);
}

// Transformation matrices
inline mat4 translationMatrix(const vec3 &translation)
{
    mat4 result(1.0f);
    result(0, 3) = translation.x;
    result(1, 3) = translation.y;
    result(2, 3) = translation.z;
    return result;
}

inline mat4 rotationMatrix(float r_angle, const vec3 &axis)
{
    float c = cosf(r_angle);
    float s = sinf(r_angle);
    float omc = 1.0f - c;
    vec3 n = normalize(axis);
    return mat4(
        n.x * n.x * omc + c, n.x * n.y * omc - n.z * s, n.x * n.z * omc + n.y * s, 0.0f,
        n.y * n.x * omc + n.z * s, n.y * n.y * omc + c, n.y * n.z * omc - n.x * s, 0.0f,
        n.z * n.x * omc - n.y * s, n.z * n.y * omc + n.x * s, n.z * n.z * omc + c, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);
}

inline mat4 scalingMatrix(const vec3 &scale)
{
    mat4 result(1.0f);
    result(0, 0) = scale.x;
    result(1, 1) = scale.y;
    result(2, 2) = scale.z;
    return result;
}

inline mat4 rotationMatrix(float pitch, float yaw, float roll) // Euler angles in radians
{
    mat4 rx = rotationMatrix(pitch, vec3(1, 0, 0));
    mat4 ry = rotationMatrix(yaw, vec3(0, 1, 0));
    mat4 rz = rotationMatrix(roll, vec3(0, 0, 1));
    return rz * ry * rx; // ZYX order
} 
