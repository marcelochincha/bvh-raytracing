#include <renderer/sr_geometry.hpp>

mesh load_ply_ascii(const std::string &filename)
{
    mesh m;
    std::ifstream plyfile(filename);
    if (!plyfile)
    {
        std::cerr << "Cant find ply file path..." << std::endl;
        exit(1);
    }

    std::string line;
    size_t num_vertices = 0;
    size_t num_faces = 0;
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
        else if (line == "end_header")
        {
            break;
        }
    }

    m.vertices.reserve(num_vertices);
    m.faces.reserve(num_faces);

    std::cout << "PLY has " << num_vertices << " vertices and " << num_faces << " faces.\n";

    std::cout << "Loading vertices...\n";
    for (size_t i = 0; i < num_vertices; ++i)
    {
        std::getline(plyfile, line);
        std::istringstream p(line);
        vertex v;

        // format is X Y Z S T
        p >> v.p.x >> v.p.y >> v.p.z >> v.t.x >> v.t.y;
        m.vertices.push_back(v);
    }

    std::cout << "Loading faces...\n";
    for (size_t i = 0; i < num_faces; ++i)
    {
        std::getline(plyfile, line);
        std::istringstream p(line);

        // Since they are index first store
        uint32_t v1, v2, v3, n;
        p >> v1 >> v2 >> v3;
        m.faces.push_back((triangle){v1, v2, v3});
    }
    std::cout << "Done\n";
    return m;
}

// Reads ONLY little endian binary PLY files with float XYZ and uint32 ST and triangular faces
mesh load_ply_binary(const std::string &filename)
{
    mesh m;
    std::ifstream plyfile(filename, std::ios::binary);

    if (!plyfile)
    {
        std::cerr << "Cant find ply file path..." << std::endl;
        exit(1);
    }

    std::string line;
    size_t num_vertices = 0;
    size_t num_faces = 0;

    // Header still ascii
    while (std::getline(plyfile, line))
    {
        if (line.find("element vertex") == 0)
        {
            sscanf(line.c_str(), "element vertex %zu", &num_vertices);
        }
        else if (line.find("element face") == 0)
        {
            sscanf(line.c_str(), "element face %zu", &num_faces);
        }
        else if (line.find("end_header") == 0)
        {
            break;
        }
    }

    std::cout << "Loading " << num_vertices << " binary vertexs..." << std::endl;
    m.vertices.resize(num_vertices);
    plyfile.read(reinterpret_cast<char *>(m.vertices.data()), num_vertices * sizeof(vertex));

    std::cout << "Loading " << num_faces << " binary faces..." << std::endl;
    m.faces.reserve(num_faces);

    for (size_t i = 0; i < num_faces; ++i)
    {
        unsigned char count;
        plyfile.read(reinterpret_cast<char *>(&count), sizeof(unsigned char));
        if (count == 3)
        {
            uint32_t indices[3];
            plyfile.read(reinterpret_cast<char *>(indices), sizeof(uint32_t) * 3);
            m.faces.push_back({indices[0], indices[1], indices[2]});
        }
    }

    std::cout << "Carga completada." << std::endl;
    return m;
}

// Generate simple meshes: cubes, planes, spheres, etc.
void create_cube(mesh* m, float size)
{
    float half = size / 2.0f;

    // Define vertices for a cube
    m->vertices = {
        {{-half, -half, -half}, {0, 0}},
        {{half, -half, -half}, {1, 0}},
        {{half, half, -half}, {1, 1}},
        {{-half, half, -half}, {0, 1}},
        {{-half, -half, half}, {0, 0}},
        {{half, -half, half}, {1, 0}},
        {{half, half, half}, {1, 1}},
        {{-half, half, half}, {0, 1}}};

    // Define faces (2 triangles per cube face)
    m->faces = {
        {0, 1, 2}, {0, 2, 3}, // Front
        {5, 4, 7},
        {5, 7, 6}, // Back
        {4, 0, 3},
        {4, 3, 7}, // Left
        {1, 5, 6},
        {1, 6, 2}, // Right
        {3, 2, 6},
        {3, 6, 7}, // Top
        {4, 5, 1},
        {4, 1, 0} // Bottom
    };
}

void create_plane(mesh* m, float width, float height)
{
    float half_w = width / 2.0f;
    float half_h = height / 2.0f;

    m->vertices = {
        {{-half_w, 0, -half_h}, {0, 0}},
        {{half_w, 0, -half_h}, {1, 0}},
        {{half_w, 0, half_h}, {1, 1}},
        {{-half_w, 0, half_h}, {0, 1}}};

    m->faces = {
        {0, 1, 2},
        {0, 2, 3}};

}

void create_sphere(mesh*m, float radius, int segments_lat, int segments_lon)
{
    for (int lat = 0; lat <= segments_lat; ++lat)
    {
        float theta = (3.14159265f * lat) / segments_lat;
        float sin_theta = std::sin(theta);
        float cos_theta = std::cos(theta);

        for (int lon = 0; lon <= segments_lon; ++lon)
        {
            float phi = (2.0f * 3.14159265f * lon) / segments_lon;
            float sin_phi = std::sin(phi);
            float cos_phi = std::cos(phi);

            vec3 pos(radius * sin_theta * cos_phi,
                     radius * cos_theta,
                     radius * sin_theta * sin_phi);

            vec2 texCoord((float)lon / segments_lon, (float)lat / segments_lat);
            m->vertices.push_back(vertex{pos, texCoord});
        }
    }

    for (int lat = 0; lat < segments_lat; ++lat)
    {
        for (int lon = 0; lon < segments_lon; ++lon)
        {
            uint32_t a = lat * (segments_lon + 1) + lon;
            uint32_t b = a + segments_lon + 1;

            m->faces.push_back({a, b, a + 1});
            m->faces.push_back({a + 1, b, b + 1});
        }
    }
}

void create_cylinder(mesh* m, float radius, float height, int segments)
{
    float half_h = height / 2.0f;

    // Create vertices
    for (int i = 0; i <= segments; ++i)
    {
        float theta = (2.0f * 3.14159265f * i) / segments;
        float x = radius * std::cos(theta);
        float z = radius * std::sin(theta);

        // Top circle
        m->vertices.push_back({{x, half_h, z}, {(float)i / segments, 1.0f}});
        // Bottom circle
        m->vertices.push_back({{x, -half_h, z}, {(float)i / segments, 0.0f}});
    }

    // Create faces
    for (int i = 0; i < segments; ++i)
    {
        int top1 = i * 2;
        int bottom1 = top1 + 1;
        int top2 = ((i + 1) % segments) * 2;
        int bottom2 = top2 + 1;

        // Side faces
        m->faces.push_back((triangle){top1, bottom1, top2});
        m->faces.push_back((triangle){top2, bottom1, bottom2});
    }
}

void create_wedge(mesh* m, float width, float height, float depth)
{
    float half_w = width / 2.0f;
    float half_h = height / 2.0f;
    float half_d = depth / 2.0f;

    // Define vertices for a wedge
    m->vertices = {
        {{-half_w, -half_h, -half_d}, {0, 0}}, // 0
        {{half_w, -half_h, -half_d}, {1, 0}},  // 1
        {{half_w, -half_h, half_d}, {1, 1}},   // 2
        {{-half_w, -half_h, half_d}, {0, 1}},  // 3
        {{-half_w, half_h, -half_d}, {0, 0}},  // 4
        {{half_w, half_h, -half_d}, {1, 0}},   // 5
        {{-half_w, half_h, half_d}, {0, 1}}    // 6
    };

    // Define faces (2 triangles per rectangular face)
    m->faces = {
        {0, 1, 2},
        {0, 2, 3}, // Bottom
        {4, 5, 1},
        {4, 1, 0}, // Back
        {3, 2, 5},
        {3, 5, 4}, // Front
        {4, 0, 3},
        {4, 3, 6}, // Left
        {1, 5, 2}, // Right (triangular face)
    };
}

mesh create_skybox_mesh()
{
    const float h = 0.5f; // Mitad del tamaño del cubo
    const float unit = 1.0f / 6.0f;
    mesh m;
    m.vertices = {
        // 0-3: FRONT (Cara 6: 5/6 a 6/6)
        {{-h, -h, -h}, {0, 0}},
        {{h, -h, -h} , {1, 0}},
        {{h, h, -h}  , {1, 1}},
        {{-h, h, -h} , {0, 1}},
        // 4-7: LEFT (Cara 1: 0/6 a 1/6)
        {{-h, -h, h} , {0, 0}},
        {{-h, -h, -h}, {1, 0}},
        {{-h, h, -h} , {1, 1}},
        {{-h, h, h}  , {0, 1}},
        // 8-11: RIGHT (Cara 2: 1/6 a 2/6)
        {{h, -h, -h} , {0, 0}},
        {{h, -h, h}  , {1, 0}},
        {{h, h, h}   , {1, 1}},
        {{h, h, -h}  , {0, 1}},
        // 12-15: BACK (Cara 5: 4/6 a 5/6)
        {{h, -h, h}  , {0, 0}},
        {{-h, -h, h} , {1, 0}},
        {{-h, h, h}  , {1, 1}},
        {{h, h, h}   , {0, 1}},
        // 16-19: TOP (Cara 3: 2/6 a 3/6)
        {{-h, h, -h} , {0, 0}},
        {{h, h, -h}  , {1, 0}},
        {{h, h, h}   , {1, 1}},
        {{-h, h, h}  , {0, 1}},
        // 20-23: BOTTOM (Cara 4: 3/6 a 4/6)
        {{-h, -h, h} , {0, 0}},
        {{h, -h, h}  , {1, 0}},
        {{h, -h, -h} , {1, 1}},
        {{-h, -h, -h}, {0, 1}},

};

    // Generar 12 triángulos (2 por cada una de las 6 caras)
    for (int i = 0; i < 24; i += 4)
    {
        m.faces.push_back({(uint32_t)i, (uint32_t)i + 1, (uint32_t)i + 2});
        m.faces.push_back({(uint32_t)i, (uint32_t)i + 2, (uint32_t)i + 3});
    }
    return m;
}