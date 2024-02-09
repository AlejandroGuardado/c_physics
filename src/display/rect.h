#ifndef _RECT_H
#define _RECT_H

#include <cglm/cglm.h>
#include <stdbool.h>

bool cwrect_init(void);
void cwrect_draw(vec3 position, vec2 scale, vec3 color, float outline, float rotation, mat4 view, mat4 projection);
void cwrect_free(void);

#endif