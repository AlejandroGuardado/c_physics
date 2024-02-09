#ifndef _COLLISION_DETECTION_H
#define _COLLISION_DETECTION_H

#include <stdbool.h>
#include <float.h>
#include <math.h>
#include "cglm/cglm.h"
#include "body.h"
#include "shape.h"
#include "contact.h"

bool cwphysics_collision_is_colliding(cwphysics_body *body_a, cwphysics_body *body_b, cwphysics_contact *contact);
bool cwphysics_collision_is_colliding_circle_circle(cwphysics_body *body_a, cwphysics_body *body_b, cwphysics_contact *contact);
bool cwphysics_collision_is_colliding_polygon_polygon(cwphysics_body *body_a, cwphysics_body *body_b, cwphysics_contact *contact);
bool cwphysics_collision_is_colliding_circle_polygon(cwphysics_body *circle, cwphysics_body *polygon, cwphysics_contact *contact);

#endif