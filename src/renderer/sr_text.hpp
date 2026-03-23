#pragma once
#include <renderer/sr_framebuffer.hpp>

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

void draw_char(framebuffer &fb, int x, int y, char c, uint32_t color, uint32_t outlineColor = 0);
void draw_text(framebuffer &fb, int x, int y, const char *text, uint32_t color, uint32_t outlineColor = 0);