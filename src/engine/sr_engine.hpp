#pragma once

#include <SDL2/SDL.h>

struct Engine;
typedef Engine Engine;

Engine* engine_create(int width, int height);
void engine_init(Engine* e);
void engine_update(Engine* e, float dt);
void engine_handle_events(Engine* e, SDL_Event& event, bool& running);
void engine_render(Engine* e, SDL_Texture* sdl_fb_texture, float dt);

#define TYPE_B
#define DEBUG_CAMERA