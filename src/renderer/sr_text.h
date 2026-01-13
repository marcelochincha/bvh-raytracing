#pragma once
#include <renderer/font8x8_basic.h>
#include <renderer/sr_primitives.hpp>

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

void draw_char(framebuffer &fb, int x, int y, char c, uint32_t color)
{
    const uint8_t *glyph = font8x8_basic[(uint8_t)c];
    //printf("Drawing char '%c' at (%d, %d)\n", c, x, y);
    for (int row = 0; row < 8; row++)
    {
        uint8_t bits = glyph[row];
        //printf(BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(bits));
        for (int col = 0; col < 8; col++)
        {
            if (x + col < 0 || x + col >= fb.width || y + row < 0 || y + row >= fb.height)
                continue;
            int target = (y + row) * fb.width + x + col;
            if (bits & (1 << col) && target >= 0 && target < fb.width * fb.height)
                fb.colorBuffer[target] = color;
        }
    }
}

void draw_text(framebuffer &fb, int x, int y, const char *text, uint32_t color)
{
    int startX = x;
    while (*text)
    {
        if (*text == '\n')
        {
            y += 8;
            x = startX;
            text++;
            continue;
        }
        draw_char(fb, x, y, *text, color);
        x += 8;
        text++;
    }
}