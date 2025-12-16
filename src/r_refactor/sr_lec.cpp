#include <iostream>
#include <vector>
#include <array>
#include <fstream>
#include <sstream>
#include <cstring>
#include "sr_math.hpp"

#define DEBUG

#ifndef STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#endif

#ifndef STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#endif

#include "stb_image_write.hpp" // Include a library for image writing, e.g., stb_image_write.h
#include "stb_image.hpp"

#define W_WIDTH 720
#define W_HEIGHT 348
#define FPS 60
#define DT (1.0f / FPS) // Delta time for each frame
#define ASPECT ((float)W_WIDTH / (float)W_HEIGHT)

// Rotation helper
mat4 get_rotation_matrix(float rX, float rY, float rZ)
{
    mat4 rmX = rotationMatrix(rX, vec3(1, 0, 0));
    mat4 rmY = rotationMatrix(rY, vec3(0, 1, 0));
    mat4 rmZ = rotationMatrix(rZ, vec3(0, 0, 1));
    return rmX * rmY * rmZ; // Combine rotations
}

// Polygon rasterization helper
typedef unsigned char byte;
typedef unsigned int pixel;

void set_pixel(int x, int y, pixel color, uint32_t *canvas)
{
    if (x < 0 || x >= W_WIDTH || y < 0 || y >= W_HEIGHT)
        return;
    canvas[y * W_WIDTH + x] = color | 0xFF000000; // Set pixel with full alpha
    //canvas[(y * W_WIDTH + x) * 3 + 2] = color & 0xFF;         // B
}

void sort_vertices_by_y(vec3 &v0, vec3 &v1, vec3 &v2)
{
    if (v1.y < v0.y)
        std::swap(v0, v1);
    if (v2.y < v0.y)
        std::swap(v0, v2);
    if (v2.y < v1.y)
        std::swap(v1, v2);
}

float interpolate_x(vec3 a, vec3 b, float y)
{
    if (a.y == b.y)
        return a.x;
    return a.x + (b.x - a.x) * ((y - a.y) / (b.y - a.y));
}
float interpolate_z(vec3 a, vec3 b, float y)
{
    if (a.y == b.y)
        return a.z;
    return a.z + (b.z - a.z) * ((y - a.y) / (b.y - a.y));
}

vec3 convert_to_screen(const vec4 &v)
{
    return vec3(
        (v.x + 1.0f) * 0.5f * (W_WIDTH - 1),
        (-v.y + 1.0f) * 0.5f * (W_HEIGHT - 1),
        v.z);
}

void fill_flat_triangle(vec3 v0, vec3 v1, vec3 v2, pixel color, uint32_t *canvas, float *depth_buffer)
{
    sort_vertices_by_y(v0, v1, v2);
    int y_start = std::ceil(v0.y);
    int y_end = std::floor(v2.y);

    if (y_start < 0)
        y_start = 0;
    if (y_end >= W_HEIGHT)
        y_end = W_HEIGHT - 1;

    for (int y = y_start; y <= y_end; ++y)
    {
        float x1, x2;
        float z1, z2;

        x1 = interpolate_x(v0, v2, y);
        z1 = interpolate_z(v0, v2, y);
        if (y < v1.y)
        {
            x2 = interpolate_x(v0, v1, y);
            z2 = interpolate_z(v0, v1, y);
        }
        else
        {
            x2 = interpolate_x(v1, v2, y);
            z2 = interpolate_z(v1, v2, y);
        }

        if (x1 > x2)
        {
            std::swap(x1, x2);
            std::swap(z1, z2);
        }

        int x_start = std::ceil(x1);
        int x_end = std::floor(x2);
        if (x_start < 0)
            x_start = 0;
        if (x_end >= W_WIDTH)
            x_end = W_WIDTH - 1;

        for (int x = x_start; x <= x_end; ++x)
        {
            // Perform depth test
            float z = z1 + (z2 - z1) * ((x - x1) / (x2 - x1));
            if (x >= 0 && x < W_WIDTH && y >= 0 && y < W_HEIGHT && z >= -1 && z <= 1 && z < depth_buffer[y * W_WIDTH + x])
            {
                set_pixel(x, y, color, canvas);
                depth_buffer[y * W_WIDTH + x] = z;
            }
        }
    }
}

struct mesh
{
    std::vector<vec3> vertices;
    std::vector<std::array<int, 3>> faces;
    std::vector<int> face_colors_index; // Optional colors per FACE
    std::vector<int> colors;            // Optional palette of colors
    bool hasColor{false};
    mat4 modelMatrix{1}; // Model intialized as a identity matrix
};

// make a ply loader
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
    if (num_colors != (size_t)-1)
    {
        m.face_colors_index.reserve(num_faces);
        m.colors.reserve(num_colors);
    }

    // Guardar vertices y caras
    for (size_t i = 0; i < num_vertices; ++i)
    {
        std::getline(plyfile, line);
        vec3 v;
        std::istringstream p(line); // Parsearlo facil
        p >> v.x >> v.y >> v.z;
        m.vertices.push_back(v);
    }

    for (size_t i = 0; i < num_faces; ++i)
    {
        std::getline(plyfile, line);
        std::istringstream p(line); // Parsearlo facil

        // Since they are index first store
        int v1, v2, v3, n;
        p >> n;
        if (num_colors != (size_t)-1)
        {
            p >> v1 >> v2 >> v3;
            int color_index;
            p >> color_index; // Read the color index
            m.faces.push_back({v1, v2, v3});
            m.face_colors_index.push_back(color_index); // Store the color index
        }
        else
        {
            p >> v1 >> v2 >> v3;
            m.faces.push_back({v1, v2, v3});
        }
    }

    // Now lastly read the colors
    if (num_colors == (size_t)-1)
        return m; // No colors to read

    for (size_t i = 0; i < num_colors; ++i)
    {
        std::getline(plyfile, line);
        std::istringstream p(line);
        int r, g, b;
        p >> r >> g >> b;
        pixel color = (r << 16) | (g << 8) | b;
        m.colors.push_back(color);
    }
    m.hasColor = (num_colors != (size_t)-1);
    return m;
}

// load texture from file

struct texture
{
    int width, height, channels;
    byte *data;
};

texture load_png(const std::string &filename)
{
    texture t;
    t.data = stbi_load(filename.c_str(), &t.width, &t.height, &t.channels, 4);
    t.channels = 4; // Force 4 channels (ARGB)
    if (!t.data)
    {
        std::cout << "Failed to load texture: " << filename << std::endl;
        exit(1);
    }

    // debug
    #ifdef DEBUG
        std::cout << "Loaded texture: " << filename << " (" << t.width << "x" << t.height << "), channels: " << t.channels << std::endl;
    #endif
    return t;
}

pixel scale_color(pixel color, float factor)
{
    // Clamp factor to [0, 1]
    if (factor < 0.0f)
        factor = 0.0f;
    if (factor > 1.0f)
        factor = 1.0f;
    int r = (color >> 16) & 0xFF;
    int g = (color >> 8) & 0xFF;
    int b = color & 0xFF;

    r = std::min(255, static_cast<int>(r * factor));
    g = std::min(255, static_cast<int>(g * factor));
    b = std::min(255, static_cast<int>(b * factor));

    return (0xFF << 24) | (r << 16) | (g << 8) | b;
}
/*
void rasterTriangle(const vec3 &v0, const vec3 &v1, const vec3 &v2, pixel color, byte *canvas, float *depth_buffer)
{
    // Transform the vertices to clip space
    vec4 mv0 = m->modelMatrix * v0;
    vec4 mv1 = m->modelMatrix * v1;
    vec4 mv2 = m->modelMatrix * v2;

    vec4 tv0 = cM * mv0;
    vec4 tv1 = cM * mv1;
    vec4 tv2 = cM * mv2;

    // Get normal for cosine shading
    vec3 edge1 = vec3(mv1 - mv0);
    vec3 edge2 = vec3(mv2 - mv0);
    vec3 face_normal = edge1.cross(edge2).normalized();
    // obtain light direction
    float l = (face_normal.dot(vec3(0, 0, -1)) + 1) * 0.5f;
    l = 0.3 + l * 0.7f; // Ambient + diffuse

    tv0 = tv0 * (1.0f / tv0.w);
    tv1 = tv1 * (1.0f / tv1.w);
    tv2 = tv2 * (1.0f / tv2.w);

    // std::cout << "Face " << i << " - TV0: " << tv0 << ", TV1: " << tv1 << ", TV2: " << tv2 << std::endl;
    //  Check if triangle is within the ndc (simple check) and frustum culling
    if (tv0.x < -1 || tv0.x > 1 || tv1.x < -1 || tv1.x > 1 || tv2.x < -1 || tv2.x > 1 ||
        tv0.y < -1 || tv0.y > 1 || tv1.y < -1 || tv1.y > 1 || tv2.y < -1 || tv2.y > 1 ||
        tv0.z < -1 || tv0.z > 1 || tv1.z < -1 || tv1.z > 1 || tv2.z < -1 || tv2.z > 1)
    {
        continue; // Skip this triangle if it's outside the NDC
    }


    vec3 sv0 = convert_to_screen(tv0);
    vec3 sv1 = convert_to_screen(tv1);
    vec3 sv2 = convert_to_screen(tv2);
    // Make random color based on face index
    pixel face_color = 0x212121;
    if (m->hasColor)
    {
        face_color = object_mesh.colors[object_mesh.face_colors_index[i]];
    }
    // face_color = scale_color(face_color, l);
    fill_flat_triangle(sv0, sv1, sv2, face_color, img_buffer, depth_buffer);
}
*/

// can stbi resize?
// TEST WITH SDL
#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>
#include <SDL2/SDL_mixer.h>
#include <stdint.h>
#include <stdbool.h>

int main()
{
    //SDL INITIALIZATION
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *window = SDL_CreateWindow(
        "sr_lec",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        W_WIDTH, W_HEIGHT, SDL_WINDOW_RESIZABLE
    );
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_RenderSetLogicalSize(renderer, W_WIDTH, W_HEIGHT);
    // Crear textura en formato RGB (32 bits por pixel)
    SDL_Texture *sdl_fb_texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        W_WIDTH, W_HEIGHT
    );
    // Framebuffer en RAM
    bool running = true;
    SDL_Event event;

    // Specificy position, rotation , and other camera parameters
    vec3 rC = vec3(0.0f, 0.0f, 0.0f); // Camera rotation
    vec3 pC = vec3(0.0f, 0.0f, 0.0f); // Camera position

    float fov = 90.0f;     // Field of view
    float aspect = ASPECT; // Aspect ratio
    float near = 0.1f;     // Near plane
    float far = 100.0f;    // Far plane

    // Camera matrix
    mat4 cam_rm = get_rotation_matrix(to_radians(rC.x), to_radians(rC.y), to_radians(rC.z)); // Rotation matrix
    mat4 cam_tm = translationMatrix(vec3(-pC.x, -pC.y, -pC.z));
    mat4 cam_rtm = cam_rm * cam_tm; // First translate, to center at camera then rotate to look at the scene

    // std::cout << "Camera rotation and translation matrix:\n" << cam_rtm << std::endl;
    //  PERSPECTIVE STEP
    float a = tanf(to_radians(fov) * 0.5f);
    float b = a * aspect;
    float c = (far + near) / (far - near);
    float d = -(2.0f * far * near) / (far - near);
    // printf("a: %f, b: %f, c: %f, d: %f\n", a, b, c, d);
    //  Create a perspective projection matrix
    //std::cout << "c:" << c << ", d:" << d << std::endl;
    mat4 psp_m(
        //  X     Y     Z     W
        1 / b, 0.0f, 0.0f, 0.0f, // col 1
        0.0f, 1 / a, 0.0f, 0.0f, // col 2
        0.0f, 0.0f, -c, -1,      // col 3
        0.0f, 0.0f, d, 0.0f);    // col 4

    // Create vector mesh
    /*
    std::vector<vec3> mesh;
    // Create a TOROIDAL MESH
    float R = 1.5, r = 1;
    for (float theta = 0; theta < 2 * M_PI; theta += M_PI / 16)
    {
        for (float phi = 0; phi < 2 * M_PI; phi += M_PI / 16)
        {
            float x = (R + r * cosf(theta)) * cosf(phi);
            float y = (R + r * cosf(theta)) * sinf(phi);
            float z = r * sinf(theta);
            mesh.push_back(vec3(x, y, z) * 0.5f); // Scale down the mesh
        }
    }*/

    /*
    // Create a simple centered cube mesh
    std::vector<vec3> verts_mesh = {
        vec3(-1, -1, -1),
        vec3(1, -1, -1),
        vec3(1, 1, -1),
        vec3(-1, 1, -1), // Back face
        vec3(-1, -1, 1),
        vec3(1, -1, 1),
        vec3(1, 1, 1),
        vec3(-1, 1, 1) // Front face
    };

    // Create faces for the cube
    std::vector<std::array<int, 3>> faces_mesh = {
        {0, 1, 2},
        {0, 2, 3}, // Back face
        {4, 5, 6},
        {4, 6, 7}, // Front face
        {0, 1, 5},
        {0, 5, 4}, // Left face
        {2, 3, 7},
        {2, 7, 6}, // Right face
        {0, 3, 7},
        {0, 7, 4}, // Top face
        {1, 2, 6},
        {1, 6, 5}  // Bottom face
    }; */

    // Create the mesh
    mesh object_mesh = load_ply("res/n64_color.ply");
    mesh floor_mesh;

    #define FLOOR_SIZE 3.0f
    // Create a simple floor
    floor_mesh.vertices = {
        vec3(FLOOR_SIZE, FLOOR_SIZE, -10),
        vec3(-FLOOR_SIZE, FLOOR_SIZE, -10),
        vec3(-FLOOR_SIZE, -FLOOR_SIZE, -10),
        vec3(FLOOR_SIZE, -FLOOR_SIZE, -10) // Back face
    };

    floor_mesh.faces = {
        {0, 1, 2},
        {0, 2, 3} // Single face for the floor
    };
    
    floor_mesh.hasColor = false;

    std::vector<mesh *> scene_meshes = {&object_mesh};
    texture skybox = load_png("res/sky.png");
    // object_mesh.vertices = verts_mesh;
    // object_mesh.faces = faces_mesh;
    //  NOW RENDER IT
    //
    uint32_t *img_buffer = new uint32_t[W_WIDTH * W_HEIGHT]; // RGB image buffer
    float *depth_buffer = new float[W_WIDTH * W_HEIGHT]; // Depth buffer for z-buffering
    float scale = 1.0f;
    // Scale the model
    // Now apply the camera matrix to each point
    vec3 rModel = vec3(0.0f, 0.0f, 0.0f); // Model rotation
    // mat4 model_rtm = get_rotation_matrix(to_radians(rModel.x), to_radians(rModel.y), to_radians(0));
    mat4 scale_m(scale);


    while(running)
    {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT)
                running = false;
        }
        // for (int j = 0; j < W_WIDTH * W_HEIGHT * 3; j++)
        //{
        //     img_buffer[j] = 0;
        // }

        // for (int y = 0; y < W_HEIGHT; ++y)
        //{
        //     for (int x = 0; x < W_WIDTH * 3; ++x)
        //     {
        //         img_buffer[y * W_WIDTH * 3 + x] = skybox.data[y * W_WIDTH * 3 + x];
        //     }
        // }

        //MAYBE RE-RENDER THE WHOLE SCENE USING A SKYBOX? BUT YOU HAVE TO HAVE TEXTURE MAPPING AH HELL NAH
        //std::memcpy(img_buffer, skybox.data, W_WIDTH * W_HEIGHT * 4); // Copy skybox as background

        for (int j = 0; j < W_WIDTH * W_HEIGHT; j++)
        {
            depth_buffer[j] = 1;
        }

        // Update model rotation for each frame - donut.c style rotation
        float A = SDL_GetTicks() * 0.04f * 0.01f;
        float B = SDL_GetTicks() * 0.02f * 0.01f;

        rModel.x = A * 180.0f / M_PI;
        rModel.z = B * 180.0f / M_PI;
        rModel.y = 0.0f;
        object_mesh.modelMatrix = (translationMatrix(vec3(0, 0, -10)) * get_rotation_matrix(to_radians(rModel.x), to_radians(rModel.y), to_radians(rModel.z)) * scale_m);
        //cam_rtm = get_rotation_matrix(to_radians(rC.x), to_radians(rC.y), to_radians(rC.z)) * cam_tm;
        //rC.x += DT * 5.0f; // Move camera around the scene
        for (mesh *m : scene_meshes)
        {
            //Now do the funny stuff - render each face
            //printf("Rendering mesh with %zu vertices and %zu faces\n", m->vertices.size(), m->faces.size());
            if (m == nullptr)
                continue;

            mat4 cM = psp_m * cam_rtm; // Combined camera matrix
            for (size_t j = 0; j < m->faces.size(); ++j)
            {
                // now get all vectors of the face
                vec3 v0 = m->vertices[m->faces[j][0]];
                vec3 v1 = m->vertices[m->faces[j][1]];
                vec3 v2 = m->vertices[m->faces[j][2]];

                // Transform the vertices to clip space
                vec4 mv0 = m->modelMatrix * v0;
                vec4 mv1 = m->modelMatrix * v1;
                vec4 mv2 = m->modelMatrix * v2;

                vec4 tv0 = cM * mv0;
                vec4 tv1 = cM * mv1;
                vec4 tv2 = cM * mv2;

                // Get normal for cosine shading
                vec3 edge1 = vec3(mv1 - mv0);
                vec3 edge2 = vec3(mv2 - mv0);
                vec3 face_normal = edge1.cross(edge2).normalized();
                // obtain light direction
                float l = 1 - (face_normal.dot(vec3(0, 0, -1)) + 1) * 0.5f;

                // #ifdef DEBUG
                //     printf("Face %zu normal: (%f, %f, %f) - Light intensity: %f\n", i, face_normal.x, face_normal.y, face_normal.z, l);
                // #endif

                // CHECK IF THE TRIANGLE IS VISIBLE checking
                // std::cout << "FRAME: " <<  i << " Face " << j << " - TV0: " << tv0 << ", TV1: " << tv1 << ", TV2: " << tv2 << std::endl;
                if (tv0.w <= 0 || tv1.w <= 0 || tv2.w <= 0)
                {
                    // std::cout << "Face " << j << " is backfacing or behind the camera. Skipping." << std::endl;
                    continue; // Skip this triangle if it's backfacing or behind the camera
                }
                tv0 = tv0 * (1.0f / tv0.w);
                tv1 = tv1 * (1.0f / tv1.w);
                tv2 = tv2 * (1.0f / tv2.w);

                //  Check if triangle is within the ndc (simple check) and frustum culling
                // if (tv0.x < -1 || tv0.x > 1 || tv1.x < -1 || tv1.x > 1 || tv2.x < -1 || tv2.x > 1 ||
                //    tv0.y < -1 || tv0.y > 1 || tv1.y < -1 || tv1.y > 1 || tv2.y < -1 || tv2.y > 1 ||
                //    tv0.z < -1 || tv0.z > 1 || tv1.z < -1 || tv1.z > 1 || tv2.z < -1 || tv2.z > 1)
                //{
                //    continue; // Skip this triangle if it's outside the NDC
                //}

                // vec4 t = psp_m * (cam_rtm * (model_rtm * mesh[i])); // Apply camera rotation and translation
                // t = t * (1.0f / t.w); // Perspective divide
                //  std::cout << "Point " << i << " = " << t << std::endl; // Print the transformed point}

                // Now convert to screen coordinates
                /*
                float screenX = (t.x + 1.0f) * 0.5f * W_WIDTH;
                float screenY = (t.y + 1.0f) * 0.5f * W_HEIGHT;
                // std::cout << "Screen coordinates of " << alfabet[i] << ": (" << screenX << ", " << screenY << ")\n";
                //  Draw the point in the image buffer
                if (screenX >= 0 && screenX < W_WIDTH && screenY >= 0 && screenY < W_HEIGHT)
                {
                    // float dist = (t.z - d) * ;
                    // printf("Point %zu: (%f, %f) - Brightness: %f\n", i, screenX, screenY, d);
                    set_pixel((int)screenX, (int)screenY, (pixel)0xFF0000, img_buffer);
                }*/

                vec3 sv0 = convert_to_screen(tv0);
                vec3 sv1 = convert_to_screen(tv1);
                vec3 sv2 = convert_to_screen(tv2);
                // Make random color based on face index
                pixel face_color = 0x212121;
                if (m->hasColor)
                {
                    // why is this being EXECUTED??????
                    // std::cout << m->colors.size() << " " << m << std::endl;
                    face_color = m->colors[m->face_colors_index[j]];
                }
                //face_color = scale_color(face_color, l);
                fill_flat_triangle(sv0, sv1, sv2, face_color, img_buffer, depth_buffer);
            }
        }
        // Convert into a file name
        //std::string filename = "out/frame_" + std::to_string(i) + ".png";
        // std::cout << "Angle_X:" << cam_rtm << std::endl;
        //  Save the image to a file
        //stbi_write_png(filename.c_str(), W_WIDTH, W_HEIGHT, 3, img_buffer, W_WIDTH * 3);
        // Subir framebuffer a la textura
        SDL_UpdateTexture(sdl_fb_texture, NULL, img_buffer, W_WIDTH * sizeof(uint32_t));

        // Renderizar en pantalla
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, sdl_fb_texture, NULL, NULL);
        SDL_RenderPresent(renderer);

        SDL_Delay(20); // Approx ~60 FPS
    }

    // Save the image
    delete[] img_buffer;
    delete[] depth_buffer;
    SDL_DestroyTexture(sdl_fb_texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}


/*
int main(int argc, char *argv[]) {
   

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
*/