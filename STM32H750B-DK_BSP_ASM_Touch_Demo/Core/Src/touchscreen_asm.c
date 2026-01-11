/**
  ******************************************************************************
  * @file    Src/touchscreen_asm.c
  * @brief   Contains touchscreen interface functions for assembly.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
extern TS_State_t  TS_State;


const uint32_t util_colors[] = {
    UTIL_LCD_COLOR_BLUE,
    UTIL_LCD_COLOR_GREEN,
    UTIL_LCD_COLOR_RED,
    UTIL_LCD_COLOR_CYAN,
    UTIL_LCD_COLOR_MAGENTA,
    UTIL_LCD_COLOR_YELLOW,
    UTIL_LCD_COLOR_LIGHTBLUE,
    UTIL_LCD_COLOR_LIGHTGREEN,
    UTIL_LCD_COLOR_LIGHTRED,
    UTIL_LCD_COLOR_LIGHTCYAN,
    UTIL_LCD_COLOR_LIGHTMAGENTA,
    UTIL_LCD_COLOR_LIGHTYELLOW,
    UTIL_LCD_COLOR_DARKBLUE,
    UTIL_LCD_COLOR_DARKGREEN,
    UTIL_LCD_COLOR_DARKRED,
    UTIL_LCD_COLOR_DARKCYAN,
    UTIL_LCD_COLOR_DARKMAGENTA,
    UTIL_LCD_COLOR_DARKYELLOW,
    UTIL_LCD_COLOR_WHITE,
    UTIL_LCD_COLOR_LIGHTGRAY,
    UTIL_LCD_COLOR_GRAY,
    UTIL_LCD_COLOR_DARKGRAY,
    UTIL_LCD_COLOR_BLACK,
    UTIL_LCD_COLOR_BROWN,
    UTIL_LCD_COLOR_ORANGE
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

// vmesnik za klic funkcij iz bsp-ja
void draw_pixel(uint16_t x, uint16_t y, uint32_t color)
{
	UTIL_LCD_SetPixel(x, y, color);
}

void draw_screen(uint32_t color)
{
	UTIL_LCD_Clear(color);
}

void draw_rect(int16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t color)
{
	UTIL_LCD_FillRect(x, y, w, h, color);

}

int touch_state(void)
{
	BSP_TS_GetState(0, &TS_State);
	if (TS_State.TouchDetected)
	{
		return 1;
	}
	return 0;
}

/*
void draw_circle(void)
{
	int x = ;
	int y = ;
	int r = ;
	uint32_t color = ;
	UTIL_LCD_FillCircle(x, y, r, color);
}

void draw_triangle(void)
{
	Triangle_Positions_t positions;
    positions.x1 = x_center;
    positions.y1 = y_center;
    positions.x2 = X2;
    positions.y2 = Y2;
    positions.x3 = X;
    positions.y3 = Y;
	UTIL_LCD_DrawPolygon(points, 3, color);
}
*/

void friNapis()
{

    UTIL_LCD_Clear(UTIL_LCD_COLOR_BLACK);
    /* Top bar */
    UTIL_LCD_FillRect(70, 50, 120, 40, UTIL_LCD_COLOR_WHITE);   /* Left */

    /* Bottom-left L */
    UTIL_LCD_FillRect(70, 110, 120, 40, UTIL_LCD_COLOR_WHITE);   /* vertical */
    UTIL_LCD_FillRect(70, 110, 40, 120, UTIL_LCD_COLOR_WHITE);    /* horizontal */

    /* Bottom-middle L */
    UTIL_LCD_FillRect(215, 110, 120, 40, UTIL_LCD_COLOR_WHITE);  /* vertical */
    UTIL_LCD_FillRect(215, 110, 40, 120, UTIL_LCD_COLOR_WHITE);   /* horizontal */

    /* Bottom-right block */
    UTIL_LCD_FillRect(360, 110, 40, 120, UTIL_LCD_COLOR_WHITE);
    UTIL_LCD_FillRect(360, 50, 40, 40, UTIL_LCD_COLOR_WHITE);

}





/**
  * @}
  */
