#ifndef _LINE_H
#define _LINE_H

#include <cglm/cglm.h>

void cwline_init(void);
void cwline_draw(vec3 v1, vec3 v2, vec3 color, float width, mat4 view, mat4 projection);
void cwline_free(void);

#endif