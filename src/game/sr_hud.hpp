#pragma once
#include <game/sr_game_state.hpp>
#include <cstdint>

double tick_ms(uint64_t& since);
void   render_raster_shadows(Game* e);
void   draw_bvh_debug(Game* e);
void   draw_normals_debug(Game* e);
void   draw_ray_debug(Game* e);
void   draw_menu(Game* e);
void   menu_apply(Game* e, int dir);
