#ifndef _CONSTRAINT_H
#define _CONSTRAINT_H

#include <cglm/cglm.h>
#include <cglm/util.h>
#include "body.h"
#include "vecn.h"
#include "matmn.h"

typedef enum{
    NO_CONSTRAINT,
    JOINT,
    PENETRATION
} cwphysics_constraint_type;

typedef struct{
    cwphysics_constraint_type type;
    cwphysics_body *body_a;
    cwphysics_body *body_b;
    cwphysics_matmn jacobian;
    vec2 point_a;
    vec2 point_b;
    vec2 normal;
    cwphysics_vecn cached_lambda;
    float bias;
    bool is_lambda_cached;
    char pad[3];
} cwphysics_constraint;

void cwphysics_constraint_set_as_joint(cwphysics_constraint *constraint, cwphysics_body *body_a, cwphysics_body *body_b, vec2 anchor_point);
void cwphysics_constraint_set_as_penetration(cwphysics_constraint *constraint, cwphysics_body *body_a, cwphysics_body *body_b, vec2 point_a, vec2 point_b, vec2 normal);

void cwphysics_constraint_get_inverse_mass(cwphysics_constraint *constraint, cwphysics_matmn* inv_mass);
void cwphysics_constraint_get_velocities(cwphysics_constraint *constraint, cwphysics_vecn* velocities);
void cwphysics_constraint_presolve(cwphysics_constraint *constraint, float dt);
void cwphysics_constraint_solve(cwphysics_constraint *constraint);
void cwphysics_constraint_postsolve(cwphysics_constraint *constraint);
void cwphysics_constraint_free(cwphysics_constraint *constraint);

#endif