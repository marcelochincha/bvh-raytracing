#define SDL_MAIN_HANDLED

#include <iostream>
#include <array>

#include <SDL2/SDL.h>

#include <math/sr_math.hpp>
#include <renderer/sr_renderer.hpp>
#include <renderer/sr_camera.hpp>
#include <renderer/sr_texture.hpp>
#include <renderer/sr_text.hpp>
#include <game/sr_game.hpp>
#include <sr_config.hpp>

SDL_Window   *window;
SDL_Renderer *renderer;
SDL_Texture  *sdl_fb_texture;

void init_sdl()
{
    SDL_Init(SDL_INIT_VIDEO);
    window = SDL_CreateWindow(
        "sr_lec",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        global_config.window_width, global_config.window_height,
        SDL_WINDOW_RESIZABLE);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_RenderSetLogicalSize(renderer, global_config.window_width, global_config.window_height);
    SDL_RenderSetIntegerScale(renderer, SDL_TRUE);
    sdl_fb_texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        global_config.window_width, global_config.window_height);

}

int main(int argc, char *argv[])
{
    global_config = parse_args(argc, argv);
    print_config(global_config);

    init_sdl();

    Game *game = game_create(global_config.window_width, global_config.window_height);
    game_init(game);

    // --bench: run the full strategy x density sweep, write CSV, and exit
    // (reproducible experiment data for the report).
    if (global_config.bench) {
        run_benchmark(game);
        game_shutdown(game);
        SDL_DestroyTexture(sdl_fb_texture);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 0;
    }

    // --demo: render the cinematic to docs/demo_XXXX.bmp (encode to mp4 after).
    if (global_config.demo) {
        run_demo(game, sdl_fb_texture);
        game_shutdown(game);
        SDL_DestroyTexture(sdl_fb_texture);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 0;
    }

    bool running = true;
    const float DT = 1.0f / 60.0f;
    float deltaTimeSeconds = DT;
    float target_delta_ms = 1000.0f / global_config.target_fps;
    uint64_t lastTime, currentTime = SDL_GetTicks64();
    int frameCount = 0;

    while (running)
    {
        lastTime = currentTime;

        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            game_handle_events(game, event, running);
        }

        if (global_config.debug_mode)
        {
            while (true)
            {
                SDL_Event debugEvent;
                if (SDL_PollEvent(&debugEvent))
                {
                    game_handle_events(game, debugEvent, running);
                    if (debugEvent.type == SDL_KEYDOWN && debugEvent.key.keysym.scancode == SDL_SCANCODE_F2)
                    {
                        deltaTimeSeconds = DT;
                        break;
                    }
                    if (debugEvent.type == SDL_QUIT)
                    {
                        running = false;
                        break;
                    }
                }
            }
        }

        if (frameCount < 5)
        {
            printf("Frame %d: dt=%.4f\n", frameCount, deltaTimeSeconds);
            frameCount++;
        }

        game_update(game, deltaTimeSeconds);
        game_render(game, sdl_fb_texture, deltaTimeSeconds);

        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, sdl_fb_texture, NULL, NULL);
        SDL_RenderPresent(renderer);

        currentTime = SDL_GetTicks64();
        if (currentTime - lastTime < target_delta_ms)
        {
            SDL_Delay((uint32_t)(target_delta_ms - (currentTime - lastTime)));
            currentTime = SDL_GetTicks64();
        }
        deltaTimeSeconds = (currentTime - lastTime) / 1000.0f;
    }

    game_shutdown(game);
    SDL_DestroyTexture(sdl_fb_texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
