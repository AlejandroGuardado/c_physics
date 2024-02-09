#ifndef _FORCE_H
#define _FORCE_H

#include "body.h"
#include "cglm/cglm.h"

void cwphysics_force_get_drag(cwphysics_body *body, vec2 force, float k);
void cwphysics_force_get_friction(cwphysics_body *body, vec2 force, float k);
void cwphysics_force_get_gravity(cwphysics_body *body_a, cwphysics_body *body_b, vec2 force, float G);
void cwphysics_force_get_spring_anchor(cwphysics_body *body, vec2 anchor, vec2 force, float rest_length, float k);
void cwphysics_force_get_spring(cwphysics_body *body_a, cwphysics_body *body_b, vec2 force, float rest_length, float k);

#endif