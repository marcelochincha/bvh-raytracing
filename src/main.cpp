#define SDL_MAIN_HANDLED

#include <iostream>
#include <vector>
#include <array>

#include <math/sr_math.hpp>
#include <renderer/sr_renderer.hpp>
#include <renderer/sr_camera.hpp>
#include <renderer/sr_texture.hpp>

//#define W_WIDTH 320
//#define W_HEIGHT 200

int main(int argc, char *argv[])
{

    std::string text_path;
    int W_WIDTH, W_HEIGHT;
    //
    if (argc >= 4)
    {
        text_path = std::string(argv[1]);
        W_WIDTH = std::stoi(argv[2]);
        W_HEIGHT = std::stoi(argv[3]);
    }
    else
    {
        printf("ARGC =%d\n", argc);
        std::cout << "Usage: " << argv[0] << " <path_to_texture> resX resY" << std::endl;
        return -1;
    }

    // Initiate the visual fb
    // First initialize the framebuffer
    framebuffer fb(W_WIDTH, W_HEIGHT);
    fb.clear(0xFF000000, 1.0f); // Clear to black and max depth

    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *window = SDL_CreateWindow(
        "sr_lec",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        W_WIDTH, W_HEIGHT, SDL_WINDOW_RESIZABLE);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_RenderSetLogicalSize(renderer, W_WIDTH, W_HEIGHT);
    SDL_Texture *sdl_fb_texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        W_WIDTH, W_HEIGHT);
    // The previous step should be handle by the engine module, sr_lec

    // INIT SCENE HERE
    // I NEED A camera
    camera cam(vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 0.0f, 0.0f), 70.0f, float(W_WIDTH) / float(W_HEIGHT), 0.01f, 100.0f);

    mesh floor_mesh = load_ply("res/skybox.ply");
    texture floor_texture;
    if (!load_png_texture(text_path, floor_texture))
    {
        return -1;
    }

    /*
    mesh floor_mesh;
    texture floor_texture;

    if (!load_png_texture(text_path, floor_texture))
    {
        return -1;
    }

    #define FLOOR_SIZE 3.0f
    // Create a simple floor
    floor_mesh.vertices = {
        {{-FLOOR_SIZE, -0.0f, -FLOOR_SIZE}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f}},
        {{FLOOR_SIZE, -0.0f, -FLOOR_SIZE}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},
        {{FLOOR_SIZE, -0.0f, FLOOR_SIZE}, {0.0f, 1.0f, 0.0f}, {1.0f, 1.0f}},
        {{-FLOOR_SIZE, -0.0f, FLOOR_SIZE}, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f}},
    };

    floor_mesh.faces = {
        {0, 1, 2},
        {0, 2, 3} // Single face for the floor
    };*/

    floor_mesh.modelMatrix = translationMatrix(vec3(0.0f, 0.0f, -5.0f)) *
                             rotationMatrix(0.0f, 0.0f, 0.0f) *
                             scalingMatrix(vec3(1.0f, 1.0f, 1.0f));
    fb.clear(0xFF000000, 1.0f);

    // NOW RENDER IT
    // Main loop, wait until window closed
    bool running = true;
    struct Player {
        vec3 position;
        vec3 rotation; // Euler angles in radians
    } player;

    while (running)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                running = false;
            }
        }


        //Player input
        const Uint8 *state = SDL_GetKeyboardState(NULL);
        float moveSpeed = 0.1f;
        if (state[SDL_SCANCODE_W])
        {
            player.position.z -= moveSpeed;
        }
        if (state[SDL_SCANCODE_S])
        {
            player.position.z += moveSpeed;
        }
        if (state[SDL_SCANCODE_A])
        {
            player.position.x -= moveSpeed;
        }
        if (state[SDL_SCANCODE_D])
        {
            player.position.x += moveSpeed;
        }


        //Now turn the camera with left and right arrows like wolf3d
        if (state[SDL_SCANCODE_LEFT])
        {
            player.rotation.y += to_radians(45.0f);
        }
        if (state[SDL_SCANCODE_RIGHT])
        {
            player.rotation.y -= to_radians(45.0f);
        }

        //Set
        cam.setPosition(player.position);
        cam.setRotation(player.rotation);

        // UPDATE ROUTINE
        //  Clear framebuffer
        fb.clear(0xFF000000, 1.0f);
        // Now traslate the main mesh every frame
        //floor_mesh.modelMatrix = translationMatrix(vec3(0.0f, 0.0f, 0.0f)) * rotationMatrix(to_radians(90.0f), SDL_GetTicks() / 1000.0f, 0.0f);
        // cam.setFov(60.0f + 30.0f * sin(SDL_GetTicks() / 1000.0f));
        //render_mesh(fb, floor_mesh, cam, 0xFF00BBFF);
        render_wireframe(fb, floor_mesh, cam, 0xFFFFFFFF);


        // tHIS SHOULD BE HANDLE ALSO BY THE ENGINE
        //  Here would go rendering code to draw into fb.colorBuffer and fb.depthBuffer
        SDL_UpdateTexture(sdl_fb_texture, NULL, fb.colorBuffer, W_WIDTH * sizeof(uint32_t));
        // Render to screen
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, sdl_fb_texture, NULL, NULL);
        SDL_RenderPresent(renderer);
        SDL_Delay(16); // Roughly ~60 FPS
    }

    // CLEAR EVERYTHING
    SDL_DestroyTexture(sdl_fb_texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
