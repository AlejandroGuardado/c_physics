#ifndef _CONTACT_H
#define _CONTACT_H

#include <stdbool.h>
#include <cglm/cglm.h>
#include "body.h"

typedef struct{
    cwphysics_body *body_a;
    cwphysics_body *body_b;
    vec2 start;
    vec2 end;
    vec2 normal;
    float depth;
    float impulse_normal_magnitude;
    float impulse_tangent_magnitude;
} cwphysics_contact;

bool cwphysics_contact_resolve_penetration(cwphysics_contact *contact);
void cwphysics_contact_resolve_collision(cwphysics_contact *contact);

#endif