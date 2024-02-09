#ifndef _VECN_H
#define _VECN_H

#include <math.h>
#include <stdlib.h>
#include "cglm/util.h"

typedef struct {
    float* vec;
    int n;
} cwphysics_vecn;

void cwphysics_vecn_get(cwphysics_vecn *v, int n);
void cwphysics_vecn_zero(cwphysics_vecn *v);
void cwphysics_vecn_copy(cwphysics_vecn *v, cwphysics_vecn *other);
float cwphysics_vecn_fetch(cwphysics_vecn *v, int index);
float* cwphysics_vecn_fetch_p(cwphysics_vecn *v, int index);
void cwphysics_vecn_set(cwphysics_vecn *v, int index, float value);
void cwphysics_vecn_add(cwphysics_vecn *a, cwphysics_vecn *b, cwphysics_vecn *dest);
void cwphysics_vecn_sub(cwphysics_vecn *a, cwphysics_vecn *b, cwphysics_vecn *dest);
void cwphysics_vecn_mul(cwphysics_vecn *a, float b, cwphysics_vecn *dest);
float cwphysics_vecn_dot(cwphysics_vecn *a, cwphysics_vecn *b);
void cwphysics_vecn_free(cwphysics_vecn *v);

#endif