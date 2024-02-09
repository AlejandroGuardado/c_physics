#include "display.h"

#include <SDL/SDL.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "glad.h"
#include "shader.h"
#include "sprite.h"
#include "line.h"
#include "circle.h"
#include "rect.h"

static SDL_Window *window = NULL;
static SDL_GLContext *context = NULL;
static SDL_Renderer *renderer = NULL;
static int window_width = WINDOW_WIDTH;
static int window_height = WINDOW_HEIGHT;
static float aspectRatio = 0.0f;
static int display_scale = 1;

bool initialize_window(mat4 view, mat4 projection){
    if(SDL_Init(SDL_INIT_VIDEO) != 0){
        fprintf(stderr, "Error initializing SDL\n");
        return false;
    }
    
    SDL_GL_LoadLibrary(NULL);
    SDL_GL_SetAttribute(SDL_GL_ACCELERATED_VISUAL, 1);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 5);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    window = SDL_CreateWindow(
        "C Physics", 
        SDL_WINDOWPOS_CENTERED, 
        SDL_WINDOWPOS_CENTERED, 
        window_width, 
        window_height, 
        SDL_WINDOW_OPENGL
    );
    if(!window){
        fprintf(stderr, "Error creating window\n");
        return false;
    }

    context = SDL_GL_CreateContext(window);
    if (context == NULL){
        fprintf(stderr, "Error creating context\n");
        return false;
    }

    gladLoadGLLoader(SDL_GL_GetProcAddress);
    SDL_GL_SetSwapInterval(1);
    glViewport(0, 0, window_width, window_height);
    glClearColor(0.39f, 0.58, 0.93f, 1.0f);
    
    glm_ortho(0.0f, HORIZONTAL_UNITS, 0.0f, VERTICAL_UNITS, 0.0f, 100.0f, projection);
    vec3 viewTranslation;
    glm_vec3_zero(viewTranslation);
    viewTranslation[0] = HORIZONTAL_UNITS / 2.0f;
    viewTranslation[1] = VERTICAL_UNITS / 2.0f;
    glm_translate(view, viewTranslation);
    
    cwsprite_init();
    cwline_init();
    cwcircle_init();
    cwrect_init();
    
    return true;
}

void swap_window(void){
    SDL_GL_SwapWindow(window);
}

void destroy_window(){
    cwsprite_free();
    cwline_free();
    cwcircle_free();
    cwrect_free();
    SDL_DestroyWindow(window);
    SDL_Quit();
}