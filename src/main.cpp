#define SDL_MAIN_HANDLED

#include <iostream>
#include <vector>
#include <array>

#include <math/sr_math.hpp>
#include <renderer/sr_renderer.hpp>
#include <renderer/sr_camera.hpp>
#include <renderer/sr_texture.hpp>
#include <renderer/sr_text.h>

// #define W_WIDTH 320
// #define W_HEIGHT 200

static inline float saturate(float x)
{
    return fminf(1.0f, fmaxf(0.0f, x));
}

float value_ramp_n(float t, int steps)
{
    t = saturate(t);

    if (steps <= 1)
        return 0.0f;

    return roundf(t * (steps - 1)) / (steps - 1);
}

int main(int argc, char *argv[])
{

    // std::string text_path, model_path;
    int W_WIDTH = 256, W_HEIGHT = 180;
    //
    // if (argc >= 4)
    //{
    //    text_path = std::string(argv[1]);
    //    model_path = std::string(argv[2]);
    //    W_WIDTH = std::stoi(argv[3]);
    //    W_HEIGHT = std::stoi(argv[4]);
    //}
    // else
    //{
    //    printf("ARGC =%d\n", argc);
    //    std::cout << "Usage: " << argv[0] << " <path_to_texture> <path_to_model> resX resY" << std::endl;
    //    return -1;
    //}

    std::cout << "Executable path: " << argv[0] << std::endl;

    // Initiate the visual fb
    // First initialize the framebuffer
    framebuffer fb(W_WIDTH, W_HEIGHT);
    fb.clear(0xFF000000, 1.0f); // Clear to black and max depth

    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *window = SDL_CreateWindow(
        "sr_lec",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        W_WIDTH, W_HEIGHT, SDL_WINDOW_RESIZABLE);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    SDL_RenderSetLogicalSize(renderer, W_WIDTH, W_HEIGHT);
    SDL_Texture *sdl_fb_texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        W_WIDTH, W_HEIGHT);
    // The previous step should be handle by the engine module, sr_lec

    // INIT SCENE HERE
    // I NEED A camera
    camera cam(vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 0.0f, 0.0f), 75.0f, float(W_WIDTH) / float(W_HEIGHT), 0.01f, 100.0f);

    pixelShaderFunc funcT = [](pixelCoord c0, void *data)
    {
        uint8_t* p = (uint8_t*)data;
        uint32_t color = *(uint32_t*)(p + 0);
        float brightness = *(float*)(p + 4);
        return brightness_color(color, brightness);
    };

    pixelShader myPXshader{
        .func = funcT,
        .data = new uint8_t[8]};

    // mesh skybox = load_ply(model_path);
    // texture skybox_texture;
    // if (!load_png_texture(text_path, skybox_texture))
    //{
    //     return -1;
    // }
    /*
    mesh floor_mesh;
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
    };
    */

    // skybox.setPosition(vec3(0.0f, 0.0f, 0.0f));
    // skybox.setRotation(vec3(SR_PI / 2.0f, 0.0f, 0.0f));
    // skybox.setScale(vec3(50.0f, 50.0f, 50.0f));

    mesh edificio = load_ply("res/skybox.ply");
    edificio.setRotation(vec3(SR_PI / 2.0f, 0.0f, 0.0f));
    fb.clear(0xFF000000, 1.0f);

    // NOW RENDER IT
    // Main loop, wait until window closed

    bool running = true;
    struct Player
    {
        vec3 position;
        float verticalRotation; // Euler angles in radians
    } player;
    // player behaves like wolf3d style
    player.position = vec3(0.0f, 0.0f, 0.0f);
    player.verticalRotation = 0.0f;

    uint64_t lastTime, currentTime = SDL_GetTicks64();
    float deltaTime = 0.0f;
    float FPS = 0.0f;
    while (running)
    {
        lastTime = currentTime;
        // FRAME START
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
                running = false;

            if (event.type == SDL_WINDOWEVENT)
                if (event.window.event == SDL_WINDOWEVENT_CLOSE)
                    running = false;
        }

        // Player input
        const Uint8 *state = SDL_GetKeyboardState(NULL);
        float moveSpeed = -4.0f;
        float turnSpeed = -to_radians(90.0f); // 90 degrees per second

        vec3 moveDirection(0.0f, 0.0f, 0.0f);
        if (state[SDL_SCANCODE_W])
        {
            moveDirection.z += moveSpeed;
        }
        if (state[SDL_SCANCODE_S])
        {
            moveDirection.z -= moveSpeed;
        }
        if (state[SDL_SCANCODE_A])
        {
            moveDirection.x += moveSpeed;
        }
        if (state[SDL_SCANCODE_D])
        {
            moveDirection.x -= moveSpeed;
        }
        if (state[SDL_SCANCODE_Q])
        {
            moveDirection.y += moveSpeed;
        }
        if (state[SDL_SCANCODE_E])
        {
            moveDirection.y -= moveSpeed;
        }

        if (state[SDL_SCANCODE_R])
        {
            player.position = vec3{};
            player.verticalRotation = 0.0f;
        }

        // Now turn the camera with left and right arrows like wolf3d
        if (state[SDL_SCANCODE_LEFT])
        {
            player.verticalRotation += turnSpeed * (1 / 60.f);
        }
        if (state[SDL_SCANCODE_RIGHT])
        {
            player.verticalRotation -= turnSpeed * (1 / 60.f);
        }

        player.verticalRotation = fmodf(player.verticalRotation, SR_PI * 2.0f);

        // Rotate movement direction based on camera rotation
        float cosY = cosf(player.verticalRotation);
        float sinY = sinf(player.verticalRotation);

        // std::cout << "player angle: " << to_degrees(player.verticalRotation) << " deg.\n";
        mat4 camWorldMat = cam.worldMatrix();
        vec3 rotatedMove = camWorldMat * vec4(moveDirection.x, moveDirection.y, moveDirection.z,0.0f);
        player.position = player.position + rotatedMove * (1 / 60.f);

        cam.setPosition(player.position);
        cam.setRotation(vec3(0.0f, player.verticalRotation, 0.0f));

        // UPDATE ROUTINE

        //  Clear framebuffer
        fb.clear(0xFF000000, 1.0f);
        // Now traslate the main mesh every frame
        // floor_mesh.modelMatrix = translationMatrix(vec3(0.0f, 0.0f, 0.0f)) * rotationMatrix(to_radians(90.0f), SDL_GetTicks() / 1000.0f, 0.0f);
        // cam.setFov(60.0f + 30.0f * sin(SDL_GetTicks() / 1000.0f));
        // render_mesh(fb, floor_mesh, cam, 0xFF00BBFF);
        // render_textured_mesh(fb, skybox, cam, skybox_texture);
        // render_mesh(fb, cam, skybox);
        //printf("Player pos: (%f, %f, %f)\n", player.position.x, player.position.y, player.position.z);
        render_mesh(fb, cam, edificio, myPXshader);
        //printf("Camera pos: (%f, %f, %f)\n", cam._position.x, cam._position.y, cam._position.z);
        render_gizmo(fb,cam,vec3(0,0,0),2.0f);
        //printf("----\n");
        // render_wireframe(fb, edificio, cam, 0xFFFFFF00);
        //  render_wireframe(fb, floor_mesh, cam, 0xFF00FFFF);

        // tHIS SHOULD BE HANDLE ALSO BY THE ENGINE
        //  Here would go rendering code to draw into fb.colorBuffer and fb.depthBuffer

        //Print to screen
        //FPS


        char HUD[64];
        snprintf(HUD, sizeof(HUD), "FPS: %.4f \nPOS: %.4f %.4f %.4f \nROT: %.4f %.4f %.4f", 1.0f / deltaTime, player.position.x, player.position.y, player.position.z, 0.0f, to_degrees(player.verticalRotation), 0.0f);
        draw_text(fb, 5,5, HUD, 0xFFFFFF00);

        SDL_UpdateTexture(sdl_fb_texture, NULL, fb.colorBuffer, W_WIDTH * sizeof(uint32_t));
        // Render to screen
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, sdl_fb_texture, NULL, NULL);
        SDL_RenderPresent(renderer);

        currentTime = SDL_GetTicks64();
        //compare delta time
        //FRAME END
        deltaTime = (currentTime - lastTime) / 1000.0f; //O(1)
        //if (currentTime % 100 < 50)
        //{
        //    printf("Delta Time: %.6f seconds\n", deltaTime);
        //    FPS = 1.0f / deltaTime;
        //}
    }

    // CLEAR EVERYTHING
    SDL_DestroyTexture(sdl_fb_texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
