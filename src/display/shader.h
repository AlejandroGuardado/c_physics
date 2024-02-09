#ifndef _SHADER_H
#define _SHADER_H

#include <stdbool.h>

typedef unsigned int shader_t;

bool shader_create(shader_t* ID, const char* vertexShaderPath, const char* fragmentShaderPath);
void shader_use(shader_t ID);
void shader_destroy(shader_t ID);
void shader_set_bool(shader_t ID, const char* name, bool value);
void shader_set_int(shader_t ID, const char* name, int value);
void shader_set_float(shader_t ID, const char* name, float value);

#endif