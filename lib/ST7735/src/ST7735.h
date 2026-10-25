#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_W 160
#define LCD_H 128

#define RGB565(r, g, b) ((uint16_t)(((uint16_t)(r) >> 3 << 11) | ((uint16_t)(g) >> 2 << 5) | ((uint16_t)(b) >> 3)))

#define POLY_BG       RGB565(148, 198, 148)
#define POLY_INK      RGB565(6, 42, 18)
#define POLY_ACCENT   RGB565(0, 148, 62)
#define POLY_CARD     RGB565(26, 88, 50)
#define POLY_CARD_TXT RGB565(214, 238, 214)
#define POLY_HI       RGB565(160, 255, 185)
#define POLY_SOFT     RGB565(175, 205, 175)
#define WHITE  RGB565(255, 255, 255)
#define BLACK  RGB565(0, 0, 0)

void LCD_Init(void);
void LCD_FillScreen(uint16_t color);
void LCD_FillRect(int x, int y, int w, int h, uint16_t color);
void LCD_DrawPixel(int x, int y, uint16_t color);
void LCD_DrawCircle(int cx, int cy, int r, uint16_t color);
void LCD_Text(int x, int y, const char* str, uint16_t fg, uint16_t bg, int size);
int LCD_TextWidth(const char* str, int size);
void LCD_TextCentered(int y, const char* str, uint16_t fg, uint16_t bg, int size);
void LCD_SevenSeg(int x, int y, int digit, int L, int T, uint16_t fg, uint16_t bg);
void LCD_PushRow(int x, int y, int w, const uint16_t* px);

#ifdef __cplusplus
}
#endif
