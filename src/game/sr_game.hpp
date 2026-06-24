#pragma once

#include <SDL2/SDL.h>

struct Game;
typedef Game Game;

Game* game_create(int width, int height);
void game_init(Game* e);
void game_update(Game* e, float dt);
void game_handle_events(Game* e, SDL_Event& event, bool& running);
void game_render(Game* e, SDL_Texture* sdl_fb_texture, float dt);
void game_shutdown(Game* e);

//#define DEBUG_CAMERA