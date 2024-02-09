#include "vecn.h"
#include <stdio.h>

void cwphysics_vecn_get(cwphysics_vecn *v, int n){
    if(n <= 0) n = 1;
    v->vec = (float*)malloc(sizeof(float) * n);
    v->n = n;
    cwphysics_vecn_zero(v);
}

void cwphysics_vecn_zero(cwphysics_vecn *v){
    for (int i = 0; i < v->n; i++){
        *(v->vec + i) = 0.0f;
    }
}

void cwphysics_vecn_copy(cwphysics_vecn *v, cwphysics_vecn *other){
    other->n = v->n;
    other->vec = (float*)malloc(sizeof(float) * v->n);
    for (int i = 0; i < v->n; i++){
        *(other->vec + i) = *(v->vec + i);
    }
}

float cwphysics_vecn_fetch(cwphysics_vecn *v, int index){
    return *(cwphysics_vecn_fetch_p(v, index));
}

float* cwphysics_vecn_fetch_p(cwphysics_vecn *v, int index){
    index = glm_clamp(0, v->n - 1, index);
    return v->vec + index;
}

void cwphysics_vecn_set(cwphysics_vecn *v, int index, float value){
    index = glm_clamp(0, v->n - 1, index);
    *(v->vec + index) = value;
}

void cwphysics_vecn_add(cwphysics_vecn *a, cwphysics_vecn *b, cwphysics_vecn *dest){
    int min = a->n < b->n ? a->n : b->n;
    min = min < dest->n ? min : dest->n;
    for (int i = 0; i < min; i++){
        *(dest->vec + i) = *(a->vec + i) + *(b->vec + i);
    }
}

void cwphysics_vecn_sub(cwphysics_vecn *a, cwphysics_vecn *b, cwphysics_vecn *dest){
    int min = a->n < b->n ? a->n : b->n;
    min = min < dest->n ? min : dest->n;
    for (int i = 0; i < min; i++){
        *(dest->vec + i) = *(a->vec + i) - *(b->vec + i);
    }
}

void cwphysics_vecn_mul(cwphysics_vecn *a, float b, cwphysics_vecn *dest){
    int min = a->n < dest->n ? a->n : dest->n;
    for (int i = 0; i < min; i++){
        *(dest->vec + i) = *(a->vec + i) * b;
    }
}

float cwphysics_vecn_dot(cwphysics_vecn *a, cwphysics_vecn *b){
    int min = a->n < b->n ? a->n : b->n;
    float sum = 0.0f;
    for (int i = 0; i < min; i++){
        sum += *(a->vec + i) + *(b->vec + i);
    }
    return sum;
}

void cwphysics_vecn_free(cwphysics_vecn *v){
    free(v->vec);
}