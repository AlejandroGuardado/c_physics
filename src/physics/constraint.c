#include "constraint.h"

void cwphysics_constraint_set_as_joint(cwphysics_constraint *constraint, cwphysics_body *body_a, cwphysics_body *body_b, vec2 anchor_point){
    constraint->type = JOINT;
    cwphysics_matmn_get(&constraint->jacobian, 1, 6);
    constraint->body_a = body_a;
    constraint->body_b = body_b;
    cwphysics_body_world_to_local(body_a, anchor_point, constraint->point_a);
    cwphysics_body_world_to_local(body_b, anchor_point, constraint->point_b);
    constraint->is_lambda_cached = false;
}

void cwphysics_constraint_set_as_penetration(cwphysics_constraint *constraint, cwphysics_body *body_a, cwphysics_body *body_b, vec2 point_a, vec2 point_b, vec2 normal){
    constraint->type = PENETRATION;
    cwphysics_matmn_get(&constraint->jacobian, 2, 6);
    constraint->body_a = body_a;
    constraint->body_b = body_b;
    cwphysics_body_world_to_local(body_a, point_a, constraint->point_a);
    cwphysics_body_world_to_local(body_b, point_b, constraint->point_b);
    cwphysics_body_world_to_local(body_a, normal, constraint->normal);
    constraint->is_lambda_cached = false;
    constraint->friction = 0.0f;
}

void cwphysics_constraint_get_inverse_mass(cwphysics_constraint *constraint, cwphysics_matmn* inv_mass){
    cwphysics_matmn_get(inv_mass, 6 , 6);
    *(cwphysics_matmn_fetch_p(inv_mass, 0, 0)) = constraint->body_a->inverseMass;
    *(cwphysics_matmn_fetch_p(inv_mass, 1, 1)) = constraint->body_a->inverseMass;
    *(cwphysics_matmn_fetch_p(inv_mass, 2, 2)) = constraint->body_a->invMomentInertia;
    *(cwphysics_matmn_fetch_p(inv_mass, 3, 3)) = constraint->body_b->inverseMass;
    *(cwphysics_matmn_fetch_p(inv_mass, 4, 4)) = constraint->body_b->inverseMass;
    *(cwphysics_matmn_fetch_p(inv_mass, 5, 5)) = constraint->body_b->invMomentInertia;
}

void cwphysics_constraint_get_velocities(cwphysics_constraint *constraint, cwphysics_vecn* velocities){
    cwphysics_vecn_get(velocities, 6);
    *(cwphysics_vecn_fetch_p(velocities, 0)) = constraint->body_a->velocity[0]; 
    *(cwphysics_vecn_fetch_p(velocities, 1)) = constraint->body_a->velocity[1]; 
    *(cwphysics_vecn_fetch_p(velocities, 2)) = constraint->body_a->angularVelocity; 
    *(cwphysics_vecn_fetch_p(velocities, 3)) = constraint->body_b->velocity[0]; 
    *(cwphysics_vecn_fetch_p(velocities, 4)) = constraint->body_b->velocity[1]; 
    *(cwphysics_vecn_fetch_p(velocities, 5)) = constraint->body_b->angularVelocity; 
}

static void presolve_joint(cwphysics_constraint *constraint, float dt){
    vec2 pa, pb;
    cwphysics_body_local_to_world(constraint->body_a, constraint->point_a, pa);
    cwphysics_body_local_to_world(constraint->body_b, constraint->point_b, pb);

    vec2 ra, rb;
    glm_vec2_sub(pa, constraint->body_a->position, ra);
    glm_vec2_sub(pb, constraint->body_b->position, rb);

    cwphysics_matmn_zero(&constraint->jacobian);
    vec2 j1, pa_minus_pb;
    glm_vec2_sub(pa, pb, pa_minus_pb);
    glm_vec2_scale(pa_minus_pb, 2.0f, j1);
    *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 0, 0)) = j1[0];
    *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 0, 1)) = j1[1];
    float j2 = 2.0f * glm_vec2_cross(ra, pa_minus_pb);
    *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 0, 2)) = j2;

    vec2 j3, pb_minus_pa;
    glm_vec2_sub(pb, pa, pb_minus_pa);
    glm_vec2_scale(pb_minus_pa, 2.0f, j3);
    *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 0, 3)) = j3[0];
    *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 0, 4)) = j3[1];
    float j4 = 2.0f * glm_vec2_cross(rb, pb_minus_pa);
    *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 0, 5)) = j4;

    float c = glm_vec2_dot(pb_minus_pa, pb_minus_pa);
    c = glm_max(0.0f, c - 0.01f);
    const float beta = 0.2f / dt;
    constraint->bias = beta * c;

    if(!constraint->is_lambda_cached) return;

    //Warm starting
    cwphysics_matmn jacobian_t;
    cwphysics_matmn_transpose(&constraint->jacobian, &jacobian_t);
    cwphysics_vecn impulses;
    cwphysics_matmn_mul_vec(&jacobian_t, &constraint->cached_lambda, &impulses);

    vec2 impulse_linear_a = { cwphysics_vecn_fetch(&impulses, 0), cwphysics_vecn_fetch(&impulses, 1) };
    vec2 impulse_linear_b = { cwphysics_vecn_fetch(&impulses, 3), cwphysics_vecn_fetch(&impulses, 4) };
    float angular_impulse_a = cwphysics_vecn_fetch(&impulses, 2);
    float angular_impulse_b = cwphysics_vecn_fetch(&impulses, 5);
    cwphysics_body_apply_impulse_linear(constraint->body_a, impulse_linear_a);
    cwphysics_body_apply_impulse_linear(constraint->body_b, impulse_linear_b);
    cwphysics_body_apply_impulse_angular(constraint->body_a, angular_impulse_a);
    cwphysics_body_apply_impulse_angular(constraint->body_b, angular_impulse_b);

    cwphysics_vecn_free(&impulses);
    cwphysics_matmn_free(&jacobian_t);
}

static void solve_joint(cwphysics_constraint *constraint){
    cwphysics_vecn velocities;
    cwphysics_constraint_get_velocities(constraint, &velocities);

    cwphysics_matmn inv_mass;
    cwphysics_constraint_get_inverse_mass(constraint, &inv_mass);

    cwphysics_matmn jacobian_t;
    cwphysics_matmn_transpose(&constraint->jacobian, &jacobian_t);

    cwphysics_vecn rhs;
    cwphysics_matmn_mul_vec(&constraint->jacobian, &velocities, &rhs);
    cwphysics_vecn_mul(&rhs, -1.0f, &rhs);
    *(cwphysics_vecn_fetch_p(&rhs, 0)) -= constraint->bias;

    cwphysics_matmn lhs_aux, lhs;
    cwphysics_matmn_mul_mat(&constraint->jacobian, &inv_mass, &lhs_aux);
    cwphysics_matmn_mul_mat(&lhs_aux, &jacobian_t, &lhs);

    cwphysics_vecn lambda, impulses;
    cwphysics_matmn_solve_gauss_seidel(&lhs, &rhs, &lambda);
    cwphysics_matmn_mul_vec(&jacobian_t, &lambda, &impulses);
    if(!constraint->is_lambda_cached){
        cwphysics_vecn_get(&constraint->cached_lambda, lambda.n);
        constraint->is_lambda_cached = true;
    }
    cwphysics_vecn_add(&constraint->cached_lambda, &lambda, &constraint->cached_lambda);

    vec2 impulse_linear_a = { cwphysics_vecn_fetch(&impulses, 0), cwphysics_vecn_fetch(&impulses, 1) };
    vec2 impulse_linear_b = { cwphysics_vecn_fetch(&impulses, 3), cwphysics_vecn_fetch(&impulses, 4) };
    float angular_impulse_a = cwphysics_vecn_fetch(&impulses, 2);
    float angular_impulse_b = cwphysics_vecn_fetch(&impulses, 5);
    cwphysics_body_apply_impulse_linear(constraint->body_a, impulse_linear_a);
    cwphysics_body_apply_impulse_linear(constraint->body_b, impulse_linear_b);
    cwphysics_body_apply_impulse_angular(constraint->body_a, angular_impulse_a);
    cwphysics_body_apply_impulse_angular(constraint->body_b, angular_impulse_b);

    cwphysics_vecn_free(&rhs);
    cwphysics_vecn_free(&velocities);
    cwphysics_vecn_free(&lambda);
    cwphysics_vecn_free(&impulses);
    cwphysics_matmn_free(&inv_mass);
    cwphysics_matmn_free(&jacobian_t);
    cwphysics_matmn_free(&lhs_aux);
    cwphysics_matmn_free(&lhs);
}

static void postsolve_joint(cwphysics_constraint *constraint){

}

static void presolve_penetration(cwphysics_constraint *constraint, float dt){
    vec2 pa, pb, n;
    cwphysics_body_local_to_world(constraint->body_a, constraint->point_a, pa);
    cwphysics_body_local_to_world(constraint->body_b, constraint->point_b, pb);
    cwphysics_body_local_to_world(constraint->body_a, constraint->normal, n);

    vec2 ra, rb;
    glm_vec2_sub(pa, constraint->body_a->position, ra);
    glm_vec2_sub(pb, constraint->body_b->position, rb);

    cwphysics_matmn_zero(&constraint->jacobian);

    vec2 inv_n;
    glm_vec2_negate_to(n, inv_n);
    *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 0, 0)) = inv_n[0];
    *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 0, 1)) = inv_n[1];
    glm_vec2_negate(ra);
    *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 0, 2)) = glm_vec2_cross(ra, n);
    *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 0, 3)) = n[0];
    *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 0, 4)) = n[1];
    *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 0, 5)) = glm_vec2_cross(rb, n);

    constraint->friction = glm_max(constraint->body_a->friction, constraint->body_b->friction);
    if(constraint->friction > 0.0f){
        vec2 t = { -n[1], n[0] };
        glm_vec2_normalize(t);
        vec2 inv_t;
        glm_vec2_negate_to(t, inv_t);
        *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 1, 0)) = inv_t[0];
        *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 1, 1)) = inv_t[1];
        *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 1, 2)) = glm_vec2_cross(ra, t);
        *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 1, 3)) = t[0];
        *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 1, 4)) = t[1];
        *(cwphysics_matmn_fetch_p(&(constraint->jacobian), 1, 5)) = glm_vec2_cross(rb, t);
    }

    vec2 va, vb;
    glm_vec2_copy(constraint->body_a->velocity, va);
    glm_vec2_copy(constraint->body_b->velocity, vb);
    //Cross product between r and w (z-axis)
    vec2 angular_a = { -constraint->body_a->angularVelocity * ra[1], constraint->body_a->angularVelocity * ra[0] };
    vec2 angular_b = { -constraint->body_b->angularVelocity * rb[1], constraint->body_b->angularVelocity * rb[0] };
    glm_vec2_add(va, angular_a, va);
    glm_vec2_add(vb, angular_b, vb);
    vec2 vrel;
    glm_vec2_sub(va, vb, vrel);
    float vrel_dot_normal = glm_vec2_dot(vrel, n);
    float e = glm_min(constraint->body_a->restitution, constraint->body_b->restitution);

    vec2 pb_minus_pa;
    glm_vec2_sub(pb, pa, pb_minus_pa);
    float c = glm_vec2_dot(pb_minus_pa, inv_n);
    c = glm_min(0.0f, c + 0.01f);
    const float beta = 0.2f / dt;
    constraint->bias = beta * c;
    constraint->bias += e * vrel_dot_normal;

    if(!constraint->is_lambda_cached) return;

    //Warm starting
    cwphysics_matmn jacobian_t;
    cwphysics_matmn_transpose(&constraint->jacobian, &jacobian_t);
    cwphysics_vecn impulses;
    cwphysics_matmn_mul_vec(&jacobian_t, &constraint->cached_lambda, &impulses);

    vec2 impulse_linear_a = { cwphysics_vecn_fetch(&impulses, 0), cwphysics_vecn_fetch(&impulses, 1) };
    vec2 impulse_linear_b = { cwphysics_vecn_fetch(&impulses, 3), cwphysics_vecn_fetch(&impulses, 4) };
    float angular_impulse_a = cwphysics_vecn_fetch(&impulses, 2);
    float angular_impulse_b = cwphysics_vecn_fetch(&impulses, 5);
    cwphysics_body_apply_impulse_linear(constraint->body_a, impulse_linear_a);
    cwphysics_body_apply_impulse_linear(constraint->body_b, impulse_linear_b);
    cwphysics_body_apply_impulse_angular(constraint->body_a, angular_impulse_a);
    cwphysics_body_apply_impulse_angular(constraint->body_b, angular_impulse_b);
    
    cwphysics_vecn_free(&impulses);
    cwphysics_matmn_free(&jacobian_t);
}

static void solve_penetration(cwphysics_constraint *constraint){
    cwphysics_vecn velocities;
    cwphysics_constraint_get_velocities(constraint, &velocities);

    cwphysics_matmn inv_mass;
    cwphysics_constraint_get_inverse_mass(constraint, &inv_mass);

    cwphysics_matmn jacobian_t;
    cwphysics_matmn_transpose(&constraint->jacobian, &jacobian_t);

    cwphysics_vecn rhs;
    cwphysics_matmn_mul_vec(&constraint->jacobian, &velocities, &rhs);
    cwphysics_vecn_mul(&rhs, -1.0f, &rhs);
    *(cwphysics_vecn_fetch_p(&rhs, 0)) -= constraint->bias;

    cwphysics_matmn lhs_aux, lhs;
    cwphysics_matmn_mul_mat(&constraint->jacobian, &inv_mass, &lhs_aux);
    cwphysics_matmn_mul_mat(&lhs_aux, &jacobian_t, &lhs);

    cwphysics_vecn lambda, impulses;
    cwphysics_matmn_solve_gauss_seidel(&lhs, &rhs, &lambda);
    if(!constraint->is_lambda_cached){
        cwphysics_vecn_get(&constraint->cached_lambda, lambda.n);
        constraint->is_lambda_cached = true;
    }
    cwphysics_vecn old_lambda;
    cwphysics_vecn_copy(&constraint->cached_lambda, &old_lambda);
    cwphysics_vecn_add(&constraint->cached_lambda, &lambda, &constraint->cached_lambda);
    float *lambda0 = cwphysics_vecn_fetch_p(&constraint->cached_lambda, 0);
    *lambda0 = *lambda0 < 0.0f ? 0.0f : *lambda0;

    if(constraint->friction > 0.0f){
        float *lambda1 = cwphysics_vecn_fetch_p(&constraint->cached_lambda, 1);
        const float max_friction = *lambda0 * constraint->friction;
        *lambda1 = glm_clamp(*lambda1, -max_friction, max_friction);
    }

    cwphysics_vecn_sub(&constraint->cached_lambda, &old_lambda, &lambda);
    cwphysics_matmn_mul_vec(&jacobian_t, &lambda, &impulses);

    vec2 impulse_linear_a = { cwphysics_vecn_fetch(&impulses, 0), cwphysics_vecn_fetch(&impulses, 1) };
    vec2 impulse_linear_b = { cwphysics_vecn_fetch(&impulses, 3), cwphysics_vecn_fetch(&impulses, 4) };
    float angular_impulse_a = cwphysics_vecn_fetch(&impulses, 2);
    float angular_impulse_b = cwphysics_vecn_fetch(&impulses, 5);
    cwphysics_body_apply_impulse_linear(constraint->body_a, impulse_linear_a);
    cwphysics_body_apply_impulse_linear(constraint->body_b, impulse_linear_b);
    cwphysics_body_apply_impulse_angular(constraint->body_a, angular_impulse_a);
    cwphysics_body_apply_impulse_angular(constraint->body_b, angular_impulse_b);

    cwphysics_vecn_free(&rhs);
    cwphysics_vecn_free(&velocities);
    cwphysics_vecn_free(&old_lambda);
    cwphysics_vecn_free(&lambda);
    cwphysics_vecn_free(&impulses);
    cwphysics_matmn_free(&inv_mass);
    cwphysics_matmn_free(&jacobian_t);
    cwphysics_matmn_free(&lhs_aux);
    cwphysics_matmn_free(&lhs);
}

static void postsolve_penetration(cwphysics_constraint *constraint){

}

void cwphysics_constraint_solve(cwphysics_constraint *constraint){
    switch (constraint->type){
        case JOINT:
            solve_joint(constraint);
            break;
        case PENETRATION:
            solve_penetration(constraint);
            break;
        default:
            return;
    }
}

void cwphysics_constraint_presolve(cwphysics_constraint *constraint, float dt){
    switch (constraint->type){
        case JOINT:
            presolve_joint(constraint, dt);
            break;
        case PENETRATION:
            presolve_penetration(constraint, dt);
            break;
        default:
            return;
    }
}

void cwphysics_constraint_postsolve(cwphysics_constraint *constraint){
    switch (constraint->type){
        case JOINT:
            postsolve_joint(constraint);
            break;
        case PENETRATION:
            postsolve_penetration(constraint);
            break;
        default:
            return;
    }
}

void cwphysics_constraint_free(cwphysics_constraint *constraint){
    cwphysics_matmn_free(&constraint->jacobian);
    if(constraint->is_lambda_cached) cwphysics_vecn_free(&constraint->cached_lambda);
}