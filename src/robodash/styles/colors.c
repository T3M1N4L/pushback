#include "liblvgl/misc/lv_color.h"

// ================================= Colors ================================= //

int hue = 200;

lv_color_t color_bg;
lv_color_t color_border;
lv_color_t color_shade;
lv_color_t color_primary;
lv_color_t color_primary_dark;
lv_color_t color_text;
lv_color_t color_red;

lv_color_t color_bar;
lv_color_t color_bar_dark;
lv_color_t color_bar_outline;

void _init_colors() {
	color_bg = lv_color_hsv_to_rgb(0, 0, 0);
	color_border = lv_color_hsv_to_rgb(0, 0, 20);
	color_shade = lv_color_hsv_to_rgb(0, 0, 20);
	color_primary = lv_color_hsv_to_rgb(266, 66, 100);
	color_primary_dark = lv_color_hsv_to_rgb(266, 66, 80);
	color_text = lv_color_hsv_to_rgb(227, 7, 100);
	color_bar = lv_color_hsv_to_rgb(0, 0, 0);
	color_bar_dark = lv_color_hsv_to_rgb(0, 0, 0);
	color_bar_outline = lv_color_hsv_to_rgb(0, 0, 20);
	color_red = lv_color_hsv_to_rgb(0, 75, 100);
}
