#include "shader.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "glad.h"

#define SHADER_LOG_SIZE 512
int shaderSuccess;
char shaderInfoLog[SHADER_LOG_SIZE];

bool read_shader_file(const char* path, char** buffer){
    FILE *fp = fopen(path, "rb");
    
    if (fp == NULL) return false;
    fseek(fp, 0L, SEEK_END);
    long size = ftell(fp);
    fclose(fp);
    
    fp = fopen(path, "r");
    char* content = memset(malloc(size + 1), '\0', size + 1);
    fread(content, 1, size, fp);
    fclose(fp);
    
    *buffer = content;

    return true;
}

bool shader_create(shader_t* ID, const char* vertexShaderPath, const char* fragmentShaderPath){
    char* vertexShaderBuffer;
    if(!read_shader_file(vertexShaderPath, &vertexShaderBuffer)){
        printf("Error reading vertex shader source\n", shaderInfoLog);
        return false;
    }

    char* fragmentShaderBuffer;
    if(!read_shader_file(fragmentShaderPath, &fragmentShaderBuffer)){
        printf("Error reading fragment shader source\n", shaderInfoLog);
        return false;
    }
    
    const char* const vertexShaderSource = vertexShaderBuffer;
    const char* const fragmentShaderSource = fragmentShaderBuffer;
    
    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, (const char**)&vertexShaderBuffer, NULL);
    glCompileShader(vertexShader);
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &shaderSuccess);
    if(!shaderSuccess){
        glGetShaderInfoLog(vertexShader, SHADER_LOG_SIZE, NULL, shaderInfoLog);
        printf("Vertex shader error: %s\n", shaderInfoLog);
        return false;
    }

    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, (const char**)&fragmentShaderBuffer, NULL);
    glCompileShader(fragmentShader);
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &shaderSuccess);
    if(!shaderSuccess){
        glGetShaderInfoLog(fragmentShader, SHADER_LOG_SIZE, NULL, shaderInfoLog);
        printf("Fragment shader error: %s\n", shaderInfoLog);
        return false;
    }

    unsigned int shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &shaderSuccess);
    if(!shaderSuccess) {
        glGetProgramInfoLog(shaderProgram, SHADER_LOG_SIZE, NULL, shaderInfoLog);
        printf("Shader program link error: %s\n", shaderInfoLog);
        return false;
    }
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    *ID = (shader_t)shaderProgram;
    return true;
}

void shader_use(shader_t ID){
    glUseProgram(ID);
}

void shader_destroy(shader_t ID){
    glDeleteProgram(ID);
}

void shader_set_bool(shader_t ID, const char* name, bool value){
    glUniform1i(glGetUniformLocation(ID, name), (int)value); 
}

void shader_set_int(shader_t ID, const char* name, int value){
    glUniform1f(glGetUniformLocation(ID, name), value);
}

void shader_set_float(shader_t ID, const char* name, float value){
    glUniform1f(glGetUniformLocation(ID, name), value); 
}