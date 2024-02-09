#ifndef _CIRCLE_H
#define _CIRCLE_H

#include <cglm/cglm.h>
#include <stdbool.h>

bool cwcircle_init(void);
void cwcircle_draw(vec3 position, float scale, vec3 color, float outline, float rotation, mat4 view, mat4 projection);
void cwcircle_free(void);

#endif