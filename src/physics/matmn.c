#include "matmn.h"
#include <stdio.h>

void cwphysics_matmn_zero(cwphysics_matmn *m){
    for (int r = 0; r < m->rows; r++){
        for (int c = 0; c < m->columns; c++){
            int i = r * m->columns;
            i += c;
            *(m->mat + i) = 0.0f;
        }
    }
}

void cwphysics_matmn_get(cwphysics_matmn *m, int rows, int columns){
    if(rows <= 0) rows = 1;
    if(columns <= 0) columns = 1;
    m->rows = rows;
    m->columns = columns;
    m->mat = (float*)malloc(sizeof(float) * m->rows * m->columns);
    cwphysics_matmn_zero(m);
}

void cwphysics_matmn_copy(cwphysics_matmn *m, cwphysics_matmn *other){
    m->rows = other->rows;
    m->columns = other->columns;
    m->mat = (float*)malloc(sizeof(float) * m->rows * m->columns);
    for (int r = 0; r < m->rows; r++){
        for (int c = 0; c < m->columns; c++){
            int i = r * m->columns;
            i += c;
            *(m->mat + i) = *(other->mat + i);
        }
    }
}

float cwphysics_matmn_fetch(cwphysics_matmn *m, int row, int column){
    return *(cwphysics_matmn_fetch_p(m, row, column));
}

float* cwphysics_matmn_fetch_p(cwphysics_matmn *m, int row, int column){
    row = glm_clamp(0, m->rows - 1, row);
    column = glm_clamp(0, m->columns - 1, column);
    int i = row * m->columns;
    i += column;
    // printf("cwphysics_matmn_fetch_p row %d column %d index %d value %f\n", row, column, i, *(m->mat + i));
    return m->mat + i;
}

void cwphysics_matmn_transpose(cwphysics_matmn *m, cwphysics_matmn *dest){
    cwphysics_matmn_get(dest, m->columns, m->rows);
    int index = 0;
    for (int c = 0; c < m->columns; c++){
        for (int r = 0; r < m->rows; r++){
            int i = r * m->columns;
            i += c;
            *(dest->mat + index) = *(m->mat + i);
            index++;
        }   
    }
}

void cwphysics_matmn_mul_vec(cwphysics_matmn *a, cwphysics_vecn *b, cwphysics_vecn *dest){
    if(b->n != a->columns){
        cwphysics_vecn_get(dest, 1);
        return;
    }
    cwphysics_vecn_get(dest, a->rows);
    for (int row = 0; row < a->rows; row++){
        *(dest->vec + row) = 0.0f;
        for (int k = 0; k < a->columns; k++){
            *(dest->vec + row) += cwphysics_matmn_fetch(a, row, k) * cwphysics_vecn_fetch(b, k);
        }
    }
}

void cwphysics_matmn_mul_mat(cwphysics_matmn *a, cwphysics_matmn *b, cwphysics_matmn *dest){
    if(a->columns != b->rows){
        cwphysics_matmn_copy(b, dest);
        return;
    }
    int num_rows = a->rows;
    int num_columns = b->columns;
    cwphysics_matmn_get(dest, num_rows, num_columns);
    for (int row = 0; row < num_rows; row++){
        for (int column = 0; column < num_columns; column++){
            int i = row * num_columns;
            i += column;
            *(dest->mat + i) = 0.0f;
            for (int k = 0; k < b->rows; k++){
                *(dest->mat + i) += cwphysics_matmn_fetch(a, row, k) * cwphysics_matmn_fetch(b, k, column);
            }
        }   
    }
}

void cwphysics_matmn_solve_gauss_seidel(cwphysics_matmn *lhs, cwphysics_vecn *rhs, cwphysics_vecn *dest){
    cwphysics_vecn_get(dest, rhs->n);
    for (int iterations = 0; iterations < rhs->n; iterations++){
        for (int i = 0; i < rhs->n; i++){
            float mat_v = cwphysics_matmn_fetch(lhs, i, i);
            
            float dot = 0.0f;
            for (int n = 0; n < rhs->n; n++){
                float mat_n = cwphysics_matmn_fetch(lhs, i, n);
                float vec_n = cwphysics_vecn_fetch(dest, n);
                dot += mat_n * vec_n;
            }
            dot /= mat_v;

            float dx = (cwphysics_vecn_fetch(rhs, i) / mat_v) - dot;
            if(isnan(dx)) continue;

            *(cwphysics_vecn_fetch_p(dest, i)) += dx;
        }   
    }
}

void cwphysics_matmn_free(cwphysics_matmn *m){
    free(m->mat);
}