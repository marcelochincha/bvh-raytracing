#pragma once
#include <game/sr_game_state.hpp>
#include <string>

vec3 color_from_hash(const std::string& name);
void build_scene_tris(Game* e);
void rebuild_field(Game* e);
