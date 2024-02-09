#ifndef _WORLD_H
#define _WORLD_H

#include <stdbool.h>
#include <cglm/cglm.h>
#include "body.h"
#include "collision_detection.h"
#include "contact.h"
#include "constraint.h"

#define BODIES_MAX_NUMBER 100
#define FORCES_MAX_NUMBER 100
#define TORQUES_MAX_NUMBER 100
#define CONTACTS_MAX_NUMBER 500
#define CONSTRAINTS_MAX_NUMBER 500
#define PENETRATIONS_MAX_NUMBER 500

typedef struct{
    float gravity;
    cwphysics_body *bodies;
    vec2 *forces;
    float *torques;
    cwphysics_contact *contacts;
    cwphysics_constraint *constraints;
    int num_forces;
    int num_torques;
    int num_contacts;
    int num_constraints;
} cwphysics_world;

void cwphysics_world_get(cwphysics_world **world, float gravity);
bool cwphysics_world_add_body(cwphysics_world *world, cwphysics_body **body, cwphysics_body_def *def);
void cwphysics_world_add_force(cwphysics_world *world, vec2 force);
void cwphysics_world_add_torque(cwphysics_world *world, float torque);
void cwphysics_world_add_and_fetch_constraint(cwphysics_world *world, cwphysics_constraint **constraint);
void cwphysics_world_update(cwphysics_world *world, float dt);
void cwphysics_world_iterate_bodies(cwphysics_world *world, void (*func)(cwphysics_body*));
void cwphysics_world_iterate_contacts(cwphysics_world *world, void (*func)(cwphysics_contact*));
void cwphysics_world_iterate_constraints(cwphysics_world *world, void (*func)(cwphysics_constraint*));
void cwphysics_world_free(cwphysics_world *world);

#endif