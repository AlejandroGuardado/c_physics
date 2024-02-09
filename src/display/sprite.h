#ifndef _SPRITE_H
#define _SPRITE_H

#include "texture.h"
#include <cglm/cglm.h>
#include <stdbool.h>

typedef struct{
    ;
    texture_t textureID;
} sprite_t;

bool cwsprite_init(void);
void cwsprite_draw(mat4 model, texture_t textureID, mat4 view, mat4 projection);
void cwsprite_free(void);

#endif