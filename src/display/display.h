#ifndef _DISPLAY_H
#define _DISPLAY_H

#include <cglm/cglm.h>
#include <stdbool.h>

#define WINDOW_WIDTH 960
#define WINDOW_HEIGHT 544

#define ASPECT_RATIO WINDOW_WIDTH/WINDOW_HEIGHT
#define VERTICAL_UNITS 10.0f
#define HORIZONTAL_UNITS VERTICAL_UNITS * ASPECT_RATIO
#define PIXELS_PER_UNIT (float)WINDOW_HEIGHT / VERTICAL_UNITS

bool initialize_window(mat4 view, mat4 projection);
void swap_window(void);
void destroy_window();

#endif