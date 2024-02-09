#ifndef _MATMN_H
#define _MATMN_H

#include <math.h>
#include <stdlib.h>
#include "vecn.h"
#include "cglm/util.h"

typedef struct{
    float *mat;
    int rows;
    int columns;
} cwphysics_matmn;

void cwphysics_matmn_get(cwphysics_matmn *m, int rows, int columns);
void cwphysics_matmn_zero(cwphysics_matmn *m);
void cwphysics_matmn_transpose(cwphysics_matmn *m, cwphysics_matmn *dest);
float cwphysics_matmn_fetch(cwphysics_matmn *m, int row, int column);
float* cwphysics_matmn_fetch_p(cwphysics_matmn *m, int row, int column);
void cwphysics_matmn_mul_vec(cwphysics_matmn *a, cwphysics_vecn *b, cwphysics_vecn *dest);
void cwphysics_matmn_mul_mat(cwphysics_matmn *a, cwphysics_matmn *b, cwphysics_matmn *dest);
void cwphysics_matmn_solve_gauss_seidel(cwphysics_matmn *lhs, cwphysics_vecn *rhs, cwphysics_vecn *dest);
void cwphysics_matmn_free(cwphysics_matmn *m);

#endif