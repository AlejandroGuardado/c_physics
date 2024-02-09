#include "force.h"

void cwphysics_force_get_drag(cwphysics_body *body, vec2 force, float k){
    glm_vec2_zero(force);
    float magnitudeSquared = glm_vec2_norm2(body->velocity);
    if(magnitudeSquared <= 0.0f) return;

    vec2 body_velocity;
    glm_vec2_normalize_to(body->velocity, body_velocity);
    glm_vec2_negate(body_velocity);

    glm_vec2_scale(body_velocity, k * magnitudeSquared, force);
}

void cwphysics_force_get_friction(cwphysics_body *body, vec2 force, float k){
    glm_vec2_zero(force);

    vec2 body_velocity;
    glm_vec2_normalize_to(body->velocity, body_velocity);
    glm_vec2_negate(body_velocity);

    glm_vec2_scale(body_velocity, k, force);
}

void cwphysics_force_get_gravity(cwphysics_body *body_a, cwphysics_body *body_b, vec2 force, float G){
    glm_vec2_zero(force);

    vec2 d;
    glm_vec2_sub(body_b->position, body_a->position, d);
    float distance_squared = glm_vec2_norm2(d);
    if(distance_squared == 0.0f) return;
    float attraction = (G * body_a->mass * body_b->mass) / distance_squared;

    float max_force = 500.0f;
    if(attraction > max_force) attraction = max_force;
    
    glm_vec2_normalize(d);
    glm_vec2_scale(d, attraction, force);
}

void cwphysics_force_get_spring_anchor(cwphysics_body *body, vec2 anchor, vec2 force, float rest_length, float k){
    glm_vec2_zero(force);
    
    vec2 spring_direction;
    glm_vec2_sub(body->position, anchor, spring_direction);
    float displacement = glm_vec2_norm(spring_direction) - rest_length;
    float spring_magnitude = -k * displacement;

    glm_vec2_normalize(spring_direction);
    glm_vec2_scale(spring_direction, spring_magnitude, force);
}

void cwphysics_force_get_spring(cwphysics_body *body_a, cwphysics_body *body_b, vec2 force, float rest_length, float k){
    cwphysics_force_get_spring_anchor(body_a, body_b->position, force, rest_length, k);
}