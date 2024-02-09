#include "contact.h"

bool cwphysics_contact_resolve_penetration(cwphysics_contact *contact){
    if(cwphysics_body_is_static(contact->body_a) && cwphysics_body_is_static(contact->body_b)) return false;
    float inv_mass_ab = contact->body_a->inverseMass + contact->body_b->inverseMass;
    float depth_ab = contact->depth / inv_mass_ab;
    float da = depth_ab * contact->body_a->inverseMass;
    float db = depth_ab * contact->body_b->inverseMass;
    vec2 normal_da, normal_db;
    glm_vec2_scale(contact->normal, da, normal_da);
    glm_vec2_scale(contact->normal, db, normal_db);
    glm_vec2_sub(contact->body_a->position, normal_da, contact->body_a->position);
    glm_vec2_add(contact->body_b->position, normal_db, contact->body_b->position);
    return true;
}

void cwphysics_contact_resolve_collision(cwphysics_contact *contact){
    cwphysics_body *body_a = contact->body_a;
    cwphysics_body *body_b = contact->body_b;
    if(!cwphysics_contact_resolve_penetration(contact)) return;

    body_a = contact->body_a;
    body_b = contact->body_b;
    vec2 ra, rb;
    glm_vec2_sub(contact->start, contact->body_a->position, ra);
    glm_vec2_sub(contact->end, contact->body_b->position, rb);
    vec2 va, vb;
    glm_vec2_copy(body_a->velocity, va);
    glm_vec2_copy(body_b->velocity, vb);
    //Cross product between r and w (z-axis)
    vec2 angular_a = { -body_a->angularVelocity * ra[1], body_a->angularVelocity * ra[0] };
    vec2 angular_b = { -body_b->angularVelocity * rb[1], body_b->angularVelocity * rb[0] };
    glm_vec2_add(va, angular_a, va);
    glm_vec2_add(vb, angular_b, vb);
    vec2 vrel;
    // glm_vec2_sub(body_a->velocity, body_b->velocity, vrel); //only linear
    glm_vec2_sub(va, vb, vrel);
    float e = fminf(body_a->restitution, body_b->restitution);
    float friction = fminf(body_a->friction, body_b->friction);
    float sum_inverse_masses = body_a->inverseMass + body_b->inverseMass;

    //Linear+angular impulse formula - along the normal
    float impulse_normal_magnitude = glm_vec2_dot(vrel, contact->normal) * -(1 + e);
    float cross_ra_n = glm_vec2_cross(ra, contact->normal);
    float cross_rb_n = glm_vec2_cross(rb, contact->normal);
    cross_ra_n *= cross_ra_n * body_a->invMomentInertia;
    cross_rb_n *= cross_rb_n * body_b->invMomentInertia;
    impulse_normal_magnitude /= sum_inverse_masses + cross_ra_n + cross_rb_n;
    contact->impulse_normal_magnitude = impulse_normal_magnitude;

    //Impulse along the tangent
    vec2 tangent = { -contact->normal[1], contact->normal[0] };
    float impulse_tangent_magnitude = friction * glm_vec2_dot(vrel, tangent) * -(1 + e);
    float cross_ra_t = glm_vec2_cross(ra, tangent);
    float cross_rb_t = glm_vec2_cross(rb, tangent);
    cross_ra_t *= cross_ra_t * body_a->invMomentInertia;
    cross_rb_t *= cross_rb_t * body_b->invMomentInertia;
    impulse_tangent_magnitude /= sum_inverse_masses + cross_ra_t + cross_rb_t;
    // printf("e %f\n", e);
    // printf("body_b->velocity %f %f\n", body_b->velocity[0], body_b->velocity[1]);
    // printf("body_b->angularVelocity %f\n", body_b->angularVelocity);
    // printf("vrel dot tangent %f\n", glm_vec2_dot(vrel, tangent));
    // printf("inv mass b %f\n", body_b->inverseMass);
    // printf("inv moment inertia b %f\n", body_b->invMomentInertia);
    // printf("ra %f %f\n", ra[0], ra[1]);
    // printf("rb %f %f\n", rb[0], rb[1]);
    // printf("sum_inverse_masses %f\n", sum_inverse_masses);
    // printf("friction %f\n", friction);
    // printf("cross_ra_t %f\n", cross_ra_t);
    // printf("cross_rb_t original %f\n", glm_vec2_cross(rb, tangent));
    // printf("cross_rb_t %f\n", cross_rb_t);
    // printf("impulse_tangent_magnitude %f\n", impulse_tangent_magnitude);
    // printf("-----------------\n", impulse_tangent_magnitude);

    vec2 impulse, impulse_normal, impulse_tangent;
    glm_vec2_scale(contact->normal, impulse_normal_magnitude, impulse_normal);
    glm_vec2_scale(tangent, impulse_tangent_magnitude, impulse_tangent);
    glm_vec2_add(impulse_normal, impulse_tangent, impulse);
    
    cwphysics_body_apply_impulse_to_point(body_a, impulse, ra);
    glm_vec2_negate(impulse);
    cwphysics_body_apply_impulse_to_point(body_b, impulse, rb);
}