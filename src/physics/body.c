#include <math.h>
#include "body.h"

void cwphysics_body_init(cwphysics_body *body, cwphysics_body_def *def){
    body->shape = def->shape;
    glm_vec2_copy(def->position, body->position);

    body->mass = def->mass;
    if(def->mass != 0.0f){
        body->inverseMass = 1.0f / def->mass;
    }
    else{
        body->inverseMass = 0.0f;
    }

    body->momentInertia = cwphysics_shape_get_moment_inertia(def->shape) * def->mass;
    if(body->momentInertia != 0.0f){
        body->invMomentInertia = 1.0f / body->momentInertia;
    }
    else{
        body->invMomentInertia = 0.0f;
    }
    
    body->restitution = 1.0f;
    body->friction = 0.7f;
    glm_vec2_zero(body->velocity);
    glm_vec2_zero(body->acceleration);
    glm_vec2_zero(body->sumForces);
    body->rotation = 0.0f;
    body->angularVelocity = 0.0f;
    body->angularAcceleration = 0.0f;
    body->sumTorque = 0.0f;
    body->render_data = NULL;
    cwphysics_shape_update_vertices(body->shape, body->position, body->rotation);
    
    body->active = false;
}

void cwphysics_body_clear(cwphysics_body *body){
    cwphysics_shape_clear(body->shape);
    body->active = false;
}

void cwphysics_body_add_force(cwphysics_body *body, vec2 force){
    glm_vec2_add(body->sumForces, force, body->sumForces);
}

void cwphysics_body_add_torque(cwphysics_body *body, float torque){
    body->sumTorque += torque;
}

void cwphysics_body_integrate_forces(cwphysics_body *body, float dt){
    if(cwphysics_body_is_static(body)) return;
    
    //Linear
    glm_vec2_scale(body->sumForces, body->inverseMass, body->acceleration);
    vec2 acceleration_dt;
    glm_vec2_scale(body->acceleration, dt, acceleration_dt);
    glm_vec2_add(body->velocity, acceleration_dt, body->velocity);
    glm_vec2_zero(body->sumForces);

    //Angular
    body->angularAcceleration = body->sumTorque * body->invMomentInertia;
    body->angularVelocity += body->angularAcceleration * dt;
    body->sumTorque = 0.0f;
}

void cwphysics_body_integrate_velocities(cwphysics_body *body, float dt){
    if(cwphysics_body_is_static(body)) return;
    
    //Linear
    vec2 velocity_dt;
    glm_vec2_scale(body->velocity, dt, velocity_dt);
    glm_vec2_add(body->position, velocity_dt, body->position);

    //Angular
    body->rotation += body->angularVelocity * dt;
}

bool cwphysics_body_is_static(cwphysics_body *body){
    const float epsilon = 0.005f;
    return fabs(body->inverseMass - 0.0f) < epsilon;
}

void cwphysics_body_apply_impulse_linear(cwphysics_body *body, vec2 impulse){
    if(cwphysics_body_is_static(body)) return;
    vec2 linear_impulse;
    glm_vec2_scale(impulse, body->inverseMass, linear_impulse);
    glm_vec2_add(body->velocity, linear_impulse, body->velocity);
}

void cwphysics_body_apply_impulse_angular(cwphysics_body *body, float impulse){
    if(cwphysics_body_is_static(body)) return;
    body->angularVelocity += impulse * body->invMomentInertia;
}

void cwphysics_body_apply_impulse_to_point(cwphysics_body *body, vec2 impulse, vec2 v_point_impact){
    if(cwphysics_body_is_static(body)) return;
    vec2 linear_impulse;
    glm_vec2_scale(impulse, body->inverseMass, linear_impulse);
    glm_vec2_add(body->velocity, linear_impulse, body->velocity);

    float angular_impulse;
    angular_impulse = glm_vec2_cross(v_point_impact, impulse);
    body->angularVelocity += angular_impulse * body->invMomentInertia;
}

void cwphysics_body_local_to_world(cwphysics_body *body, vec2 local, vec2 world){
    glm_vec2_rotate(local, body->rotation, world);
    glm_vec2_add(world, body->position, world);
}

void cwphysics_body_world_to_local(cwphysics_body *body, vec2 world, vec2 local){
    glm_vec2_sub(world, body->position, local);
    glm_vec2_rotate(local, -body->rotation, local);
}