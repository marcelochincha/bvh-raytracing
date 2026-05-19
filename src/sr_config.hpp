#pragma once

#include <cstring>
#include <cstdlib>
#include <iostream>

#define W_WIDTH 480
#define W_HEIGHT 240
#define TARGET_FPS 60.0f
#define AUDIO_RATE 8192 * 2
#define DEBUG false

struct config
{
    int window_width;
    int window_height;
    float target_fps;
    int audio_rate;
    bool debug_mode;

    config()
        : window_width(W_WIDTH),
          window_height(W_HEIGHT),
          target_fps(TARGET_FPS),
          audio_rate(AUDIO_RATE),
          debug_mode(DEBUG)
    {
    }
};

inline void print_help()
{
    printf("SR-LEC Usage:\n");
    printf("  --width <pixels>        Window width (default: 320)\n");
    printf("  --height <pixels>       Window height (default: 200)\n");
    printf("  --fps <value>           Target FPS (default: 60.0)\n");
    printf("  --audio-rate <hz>       Audio sample rate (default: 16384)\n");
    printf("  --debug                 Enable debug mode\n");
    printf("  --help                  Show this help\n");
    printf("\nExample: sr_lec.exe --width 320 --height 200  --fps 60\n");
}

inline config parse_args(int argc, char* argv[])
{
    config cfg;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--width") == 0 && i + 1 < argc)
        {
            cfg.window_width = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--height") == 0 && i + 1 < argc)
        {
            cfg.window_height = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--fps") == 0 && i + 1 < argc)
        {
            cfg.target_fps = (float)atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--audio-rate") == 0 && i + 1 < argc)
        {
            cfg.audio_rate = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--debug") == 0)
        {
            cfg.debug_mode = true;
        }
        else if (strcmp(argv[i], "--help") == 0)
        {
            print_help();
            exit(0);
        }
    }

    return cfg;
}

inline void print_config(const config& cfg)
{
    printf("Configuration loaded:\n");
    printf("  Window: %dx%d\n", cfg.window_width, cfg.window_height);
    printf("  Target FPS: %.1f\n", cfg.target_fps);
    printf("  Audio Rate: %d Hz\n", cfg.audio_rate);
    printf("  Debug Mode: %s\n", cfg.debug_mode ? "ON" : "OFF");
}

inline config global_config;
