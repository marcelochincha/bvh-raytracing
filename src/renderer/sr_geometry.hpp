#pragma once

#include <math/sr_math.hpp>
#include <cstdint>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

struct vertex
{
    vec3 p;
    vec2 t;
};

struct triangle
{
    uint32_t v0, v1, v2;
};

// Simple array structure with each face as a triangle (3 vertex indices)
// In the order of faces : LEFT, RIGHT, TOP, BOTTOM, BACK, FRONT
typedef std::array<vertex, 24> raw_skybox_mesh;

struct mesh
{
    // Mesh data
    std::vector<vertex> vertices;
    std::vector<triangle> faces;

    // Model transformation
    mutable mat4 _modelMatrix = mat4(1.0f);
    mutable bool _modelMatrixDirty = true;
    vec3 position = vec3(0.0f, 0.0f, 0.0f);
    vec3 rotation = vec3(0.0f, 0.0f, 0.0f);
    vec3 scale = vec3(1.0f, 1.0f, 1.0f);
    bool inverseFaces = false; // Whether to invert normals for lighting calculations

    void recalculateModelMatrix() const
    {
        mat4 t = translationMatrix(position);
        mat4 r = rotationMatrix(rotation.x, rotation.y, rotation.z);
        mat4 s = scalingMatrix(scale);
        _modelMatrix = t * r * s;
    }

    const mat4 &modelMatrix() const
    {
        if (_modelMatrixDirty)
            recalculateModelMatrix();
        return _modelMatrix;
    }

    mesh() {}

    // Setters that mark dirty
    void setPosition(const vec3 &pos)
    {
        position = pos;
        _modelMatrixDirty = true;
    }

    void setRotation(const vec3 &rot)
    {
        rotation = rot;
        _modelMatrixDirty = true;
    }

    void setScale(const vec3 &s)
    {
        scale = s;
        _modelMatrixDirty = true;
    }

    // Convenience method to update rotation by adding deltas
    void updateRotation(const vec3 &deltaRot)
    {
        rotation = rotation + deltaRot;
        _modelMatrixDirty = true;
    }

};

mesh load_ply_ascii(const std::string &filename);
mesh load_ply_binary(const std::string &filename);


// Generate simple meshes: cubes, planes, spheres, etc.
void create_cube(mesh*m, float size);
void create_plane(mesh*m, float width, float height);
void create_sphere(mesh*m, float radius, int segments_lat, int segments_lon);
void create_cylinder(mesh*m, float radius, float height, int segments);
void create_wedge(mesh*m, float width, float height, float depth);


mesh create_skybox_mesh();