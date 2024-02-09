#ifndef _BODY_H
#define _BODY_H

#include <cglm/cglm.h>
#include <stdbool.h>
#include "shape.h"



typedef struct {
    vec2 position;
    vec2 velocity;
    vec2 acceleration;
    vec2 sumForces;
    float mass;
    float inverseMass;
    float rotation;
    float angularVelocity;
    float angularAcceleration;
    float momentInertia;
    float invMomentInertia;
    float sumTorque;
    float restitution;
    float friction;
    cwphysics_shape *shape;
    void* render_data;
    bool active;
    char pad[3];
} cwphysics_body;

typedef struct {
    cwphysics_shape *shape;
    vec2 position;
    float mass;
} cwphysics_body_def;

void cwphysics_body_init(cwphysics_body *body, cwphysics_body_def *def);
void cwphysics_body_clear(cwphysics_body *body);
void cwphysics_body_add_force(cwphysics_body *body, vec2 force);
void cwphysics_body_add_torque(cwphysics_body *body, float torque);
void cwphysics_body_apply_impulse_linear(cwphysics_body *body, vec2 impulse);
void cwphysics_body_apply_impulse_angular(cwphysics_body *body, float impulse);
void cwphysics_body_apply_impulse_to_point(cwphysics_body *body, vec2 impulse, vec2 v_point_impact);
void cwphysics_body_integrate_forces(cwphysics_body *body, float dt);
void cwphysics_body_integrate_velocities(cwphysics_body *body, float dt);
void cwphysics_body_iterate(void (*func)(cwphysics_body*, void*), void* payload);
void cwphysics_body_iterate_dt(void (*func)(cwphysics_body*, float, void*), float dt, void* payload);
void cwphysics_body_iterate_eachother_payload(void (*func)(cwphysics_body*, cwphysics_body*, void*), void* payload);
bool cwphysics_body_is_static(cwphysics_body *body);
void cwphysics_body_local_to_world(cwphysics_body *body, vec2 local, vec2 world);
void cwphysics_body_world_to_local(cwphysics_body *body, vec2 world, vec2 local);

#endif