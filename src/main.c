#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#define SDL_MAIN_HANDLED
#include <SDL/SDL.h>
#include "glad.h"

#include <cglm/cglm.h>
#include "upng.h"
#include "display/display.h"
#include "display/circle.h"
#include "display/rect.h"
#include "display/line.h"
#include "display/sprite.h"

#include "physics/body.h"
#include "physics/world.h"
#include "physics/constraint.h"

static mat4 viewMatrix;
static mat4 projectionMatrix;
static vec3 cameraPosition;
int mouse_x, mouse_y;
vec2 mouse_position;
bool is_running = false;
bool is_debug = false;

cwphysics_world *world;
texture_t crate_texture;
// texture_t basketball_texture;
// texture_t bowlingball_texture;

bool setup(void){
    glm_mat4_identity(viewMatrix);
    glm_vec3_zero(cameraPosition);
    cameraPosition[2] -= 50.0f;
    glm_translate(viewMatrix, cameraPosition);

    cwphysics_world_get(&world, -9.81f);
    if(!initialize_window(viewMatrix, projectionMatrix)){
        cwphysics_world_free(world);
        return false;
    }
    
    if(!cwtexture_load(&crate_texture, "assets/textures/crate.png")){
        fprintf(stderr, "crate texture not loaded");
        return false;
    }

    cwphysics_body *b;
    cwphysics_body_def def;

    vec2 pos = { 0.0f, 0.0f };
    
    cwphysics_body *body;
    cwphysics_shape *circle_2;
    cwphysics_shape_get_circle(&circle_2, 3.0f);
    def.shape = circle_2;
    pos[0] = 0.0f;
    pos[1] = 0.0f;
    glm_vec2_copy(pos, def.position);
    def.mass = 0.0f;
    cwphysics_world_add_body(world, &body, &def);

    // cwphysics_body *body;
    // cwphysics_shape *box;
    // pos[0] = 0.0f;
    // pos[1] = 0.0f;
    // cwphysics_shape_get_box(&box, 2.5f, 2.5f);
    // def.shape = box;
    // glm_vec2_copy(pos, def.position);
    // def.mass = 0.0f;
    // cwphysics_world_add_body(world, &body, &def);
    // body->rotation = 1.4f;
    // body->restitution = 0.8f;
    // body->friction = 1.0f;

    return true;
}

void process_input(void){
    SDL_Event event;
    SDL_PollEvent(&event);
    switch (event.type){
        case SDL_QUIT:
            is_running = false;
            break;
        case SDL_KEYDOWN:
            if(event.key.keysym.sym == SDLK_TAB){
                is_debug = !is_debug;
            }
            if(event.key.keysym.sym == SDLK_ESCAPE){
                is_running = false;
            }
            break;
        case SDL_KEYUP:
            if(event.key.keysym.sym == SDLK_1){
                vec2 f = { 1.1f, 0.0f };
                cwphysics_world_add_force(world, f);
            }
            break;
        case SDL_MOUSEBUTTONDOWN:
            //Spawn circle

            // cwphysics_shape *circle;
            // bool t_random = 0.5f < (float)rand() / RAND_MAX;
            // // float size = 0.35f;
            // float size = t_random ? 0.5f  : 0.35f;
            // if(cwphysics_shape_get_circle(&circle, size)){                    
            //     // float mass = 1.0f;
            //     float mass = t_random ? 1.0f : 2.5f;
            //     cwphysics_body *body;
            //     cwphysics_body_def def;
            //     def.shape = circle;
            //     glm_vec2_copy(mouse_position, def.position);
            //     def.mass = mass;
            //     if(cwphysics_world_add_body(world, &body, &def)){
            //         // body->restitution = 0.4f;
            //         body->restitution = t_random ? 0.8f : 0.4f;
            //         body->friction = t_random ? 0.1f : 0.8f;
            //         // body->render_data = t_random ? &basketball_texture : &bowlingball_texture;
            //     }
            // }

            //Spawn box

            cwphysics_shape *box;
            if(cwphysics_shape_get_box(&box, 1.0f, 1.0f)){
                cwphysics_body_def def;
                def.shape = box;
                glm_vec2_copy(mouse_position, def.position);
                def.mass = 3.0f;
                cwphysics_body *body;
                if(cwphysics_world_add_body(world, &body, &def)){
                    body->restitution = 0.1f;
                    body->friction = 0.01f;
                    // body->render_data = &crate_texture;
                }
            }

            //Spawn polygon
            
            // vec2 *positions = malloc(sizeof(vec2) * 5);
            // (*positions)[0] = 0.0f;
            // (*positions)[1] = 0.8506f;
            // (*(positions + 1))[0] = 0.8090f;
            // (*(positions + 1))[1] = 0.2628;
            // (*(positions + 2))[0] = 0.5;
            // (*(positions + 2))[1] = -0.6882;
            // (*(positions + 3))[0] = -0.5;
            // (*(positions + 3))[1] = -0.6882;
            // (*(positions + 4))[0] = -0.8090f;
            // (*(positions + 4))[1] = 0.2628;

            // cwphysics_shape *polygon;
            // cwphysics_body *b;
            // cwphysics_shape_get_polygon(&polygon, positions, 5, 0.5f);
            // cwphysics_body_get(&b, polygon, mouse_position, 1.0f);
            // b->restitution = 0.7f;
            // b->friction = 0.7f;
            break;
    }
    SDL_GetMouseState(&mouse_x, &mouse_y);
}

void update(double delta_time){
    cwphysics_world_update(world, delta_time);

    // glm_vec2_copy(mouse_position, test->position);

    mouse_position[0] = (float)mouse_x / (PIXELS_PER_UNIT);
    mouse_position[1] = (float)mouse_y / (PIXELS_PER_UNIT);
    vec2 camera_center = { viewMatrix[3][0], viewMatrix[3][1] };
    glm_vec2_sub(mouse_position, camera_center, mouse_position);
    mouse_position[1] *= -1.0f;
}

void draw_body(cwphysics_body *body){
    bool contains_render_data = body->render_data != NULL;
    if(contains_render_data){
        sprite_t sprite;
        texture_t textureID = *((texture_t*)(body->render_data));
        vec2 scale = { 1.0f, 1.0f };
        switch (body->shape->type){
            case BOX:
                glm_vec2_copy(((cwphysics_shape_box*)(body->shape->data))->polygon.scale, scale);
                break;
            case POLYGON:
                glm_vec2_copy(((cwphysics_shape_box*)(body->shape->data))->polygon.scale, scale);
                break;
            case CIRCLE:
                float scale_c = ((cwphysics_shape_circle*)(body->shape->data))->radius * 2.0f;
                scale[0] = scale_c;
                scale[1] = scale_c;
                break;
            default:
                break;
        }
        mat4 model;
        glm_mat4_identity(model);

        vec3 translation_3d = { body->position[0], body->position[1], 0.0f };
        glm_translate(model, translation_3d);

        glm_rotate_z(model, body->rotation, model);

        vec3 scale_3d = { scale[0], scale[1], 1.0f };
        glm_scale(model, scale_3d);
        
        cwsprite_draw(model, textureID, viewMatrix, projectionMatrix);

        if(!is_debug) return;
    }

    vec3 body_position = { body->position[0], body->position[1], 0.0f };
    vec3 color = { 1.0f, 1.0f, 1.0f };

    cwphysics_shape_polygon *polygon_data;
    int numVertices;

    switch (body->shape->type){
        case CIRCLE:
            float scale = ((cwphysics_shape_circle*)(body->shape->data))->radius * 2.0f;
            cwcircle_draw(body_position, scale, color, 0.05f, body->rotation, viewMatrix, projectionMatrix);
            return;
        case BOX:
        case POLYGON:
            polygon_data = ((cwphysics_shape_polygon*)(body->shape->data));
            numVertices = polygon_data->numVertices;
            break;
        default:
            return;
    }

    vec2 *vertices = polygon_data->transformedVertices;
    for(int i = 0; i < numVertices; i++){
        vec2 p1;
        glm_vec2_copy(*(vertices + i), p1);

        int next = i + 1;
        next %= numVertices;
        vec2 p2;
        glm_vec2_copy(*(vertices + next), p2);
        vec3 p1_3d = { p1[0], p1[1], 0.0f };
        vec3 p2_3d = { p2[0], p2[1], 0.0f };

        cwline_draw(p1_3d, p2_3d, color, 0.2f, viewMatrix, projectionMatrix);
    }
}

static void render_contact(cwphysics_contact *contact){
    vec3 color = { 1.0f, 1.0f, 1.0f };
    vec2 tangent = { -contact->normal[1], contact->normal[0] };
    vec3 start_pos = { contact->start[0], contact->start[1], 1.0f };
    cwcircle_draw(start_pos, 0.1f, color, 1.0f, 0.0f, viewMatrix, projectionMatrix);
    vec3 end_pos;
    // glm_vec2_scale(tangent, contact.impulse_tangent_magnitude, tangent);
    glm_vec2_add(start_pos, tangent, end_pos);
    color[2] = 0.0f;
    cwline_draw(start_pos, end_pos, color, 2.0f, viewMatrix, projectionMatrix);
    glm_vec2_add(start_pos, contact->normal, end_pos);
    color[0] = 0.0f;
    cwline_draw(start_pos, end_pos, color, 2.0f, viewMatrix, projectionMatrix);
    glm_vec2_copy(contact->end, end_pos);
    color[0] = 1.0f;
    color[1] = 0.0f;
    color[2] = 0.0f;
    cwcircle_draw(end_pos, 0.1f, color, 1.0f, 0.0f, viewMatrix, projectionMatrix);
}

static void render_constraint(cwphysics_constraint *constraint){
    vec3 color = { 1.0f, 0.4f, 0.4f };
    vec2 point_a, point_b;
    cwphysics_body_local_to_world(constraint->body_a, constraint->point_a, point_a);
    cwphysics_body_local_to_world(constraint->body_b, constraint->point_b, point_b);
    vec3 point_a_3d = { point_a[0], point_a[1], 0.0f };
    vec3 point_b_3d = { point_b[0], point_b[1], 0.0f };
    cwline_draw(point_a_3d, point_b_3d, color, 2.0f, viewMatrix, projectionMatrix);
}

void render(void){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_CULL_FACE);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    cwphysics_world_iterate_bodies(world, draw_body);
    if(is_debug) {
        cwphysics_world_iterate_contacts(world, render_contact);
        cwphysics_world_iterate_constraints(world, render_constraint);
    }

    swap_window();
}

static void free_physics(cwphysics_body *body){
    cwphysics_body_clear(body);
}

void free_resources(void){
    cwphysics_world_free(world);
    destroy_window();
}

int main(int argc, char *argv[]){
    is_running = setup();
    
    SDL_SetRelativeMouseMode(SDL_FALSE);

    Uint32 game_time = SDL_GetTicks();
    double fps_cap = 1.0 / 30.0;
    while(is_running){
        Uint32 now = SDL_GetTicks();
        double delta_time = (now - game_time) / 1000.0;
        if(delta_time > fps_cap){
            delta_time = fps_cap;
        }
        game_time = now;
        
        mouse_x = 0, mouse_y = 0;
        
        process_input();
        update(delta_time),
        render();
    }
    free_resources();

    return 0;
}