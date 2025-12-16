#pragma once

#include <math/sr_math.hpp>
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

struct vertex {
    vec3 p;
    vec3 n;
    vec2 t;
};

struct triangle {
    uint32_t v0, v1, v2;
};

struct mesh {
    std::vector<vertex> vertices;
    std::vector<triangle> faces;
    mat4 modelMatrix = mat4(1.0f);
};

mesh load_ply(const std::string &filename)
{
    mesh m;
    std::ifstream plyfile(filename);
    if (!plyfile)
    {
        std::cout << "Cant find ply file path..." << std::endl;
        exit(1);
    }

    std::string line;
    size_t num_vertices = 0;
    size_t num_faces = 0;
    size_t num_colors = -1;
    while (std::getline(plyfile, line))
    {
        if (line.rfind("element vertex", 0) == 0)
        {
            std::sscanf(line.c_str(), "element vertex %zu", &num_vertices);
        }
        else if (line.rfind("element face", 0) == 0)
        {
            std::sscanf(line.c_str(), "element face %zu", &num_faces);
        }
        else if (line.rfind("element color", 0) == 0)
        {
            std::sscanf(line.c_str(), "element color %zu", &num_colors);
        }
        else if (line == "end_header")
        {
            break;
        }
    }

    m.vertices.reserve(num_vertices);
    m.faces.reserve(num_faces);
    // if (num_colors != (size_t)-1)
    //{
    //     m.face_colors_index.reserve(num_faces);
    //     m.colors.reserve(num_colors);
    // }

    // Guardar vertices y caras
    for (size_t i = 0; i < num_vertices; ++i)
    {
        std::getline(plyfile, line);
        std::istringstream p(line); // Parsearlo facil
        vertex v;

        // format is X Y Z S T
        p >> v.p.x >> v.p.y >> v.p.z >> v.t.x >> v.t.y;
        m.vertices.push_back(v);
    }

    for (size_t i = 0; i < num_faces; ++i)
    {
        std::getline(plyfile, line);
        std::istringstream p(line); // Parsearlo facil

        // Since they are index first store
        uint32_t v1, v2, v3, n;
        p >> n;
        if (num_colors != (size_t)-1)
        {
            p >> v1 >> v2 >> v3;
            int color_index;
            p >> color_index; // Read the color index
            m.faces.push_back((triangle){v1, v2, v3});
            // m.face_colors_index.push_back(color_index); // Store the color index
        }
        else
        {
            p >> v1 >> v2 >> v3;
            m.faces.push_back((triangle){v1, v2, v3});
        }
    }

    // Now lastly read the colors
    // if (num_colors == (size_t)-1)
    //    return m; // No colors to read
    //
    // for (size_t i = 0; i < num_colors; ++i)
    //{
    //    std::getline(plyfile, line);
    //    std::istringstream p(line);
    //    int r, g, b;
    //    p >> r >> g >> b;
    //    pixel color = (r << 16) | (g << 8) | b;
    //    m.colors.push_back(color);
    //}
    // m.hasColor = (num_colors != (size_t)-1);
    return m;
}