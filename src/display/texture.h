#ifndef _TEXTURE_H
#define _TEXTURE_H

#include "upng.h"
#include <stdbool.h>

typedef unsigned int texture_t;
bool cwtexture_load(texture_t* texture, const char* filename);

#endif