#include "world.h"

void cwphysics_world_get(cwphysics_world **world, float gravity){
    cwphysics_world *aux = malloc(sizeof(cwphysics_world));
    aux->gravity = gravity;
    aux->num_forces = 0;
    aux->num_torques = 0;
    aux->num_contacts = 0;
    aux->num_joint_constraints = 0;
    aux->num_penetration_constraints = 0;

    aux->bodies = (cwphysics_body*)malloc(sizeof(cwphysics_body) * BODIES_MAX_NUMBER);
    aux->forces = (vec2*)malloc(sizeof(vec2) * FORCES_MAX_NUMBER);
    aux->torques = (float*)malloc(sizeof(float) * TORQUES_MAX_NUMBER);
    aux->contacts = (cwphysics_contact*)malloc(sizeof(cwphysics_contact) * CONTACTS_MAX_NUMBER);
    aux->joint_constraints = (cwphysics_constraint*)malloc(sizeof(cwphysics_constraint) * JOINT_CONSTRAINTS_MAX_NUMBER);
    aux->penetration_constraints = (cwphysics_constraint*)malloc(sizeof(cwphysics_constraint) * PENETRATION_CONSTRAINTS_MAX_NUMBER);

    *world = aux;
}

bool cwphysics_world_add_body(cwphysics_world *world, cwphysics_body **body, cwphysics_body_def *def){
    for (int i = 0; i < BODIES_MAX_NUMBER; i++){
        cwphysics_body *aux = world->bodies + i;
        if(aux->active) continue;
        cwphysics_body_init(aux, def);
        aux->active = true;
        *body = aux;
        return true;
    }
    return false;
}

void cwphysics_world_add_force(cwphysics_world *world, vec2 force){
    int index = world->num_forces;
    if(index >= FORCES_MAX_NUMBER) return;
    glm_vec2_copy(force, *(world->forces + index));
    world->num_forces++;
}

void cwphysics_world_add_torque(cwphysics_world *world, float torque){
    int index = world->num_torques;
    if(index >= TORQUES_MAX_NUMBER) return;
    *(world->torques + index) = torque;
    world->num_torques++;
}

void cwphysics_world_add_and_fetch_joint_constraint(cwphysics_world *world, cwphysics_constraint **constraint, cwphysics_body *body_a, cwphysics_body *body_b, vec2 anchor_point){
    int index = world->num_joint_constraints;
    if(index >= JOINT_CONSTRAINTS_MAX_NUMBER) {
        return;
    }
    *constraint = world->joint_constraints + index;
    cwphysics_constraint_set_as_joint(*constraint, body_a, body_b, anchor_point);
    world->num_joint_constraints++;
}

void cwphysics_world_add_and_fetch_penetration_constraint(cwphysics_world *world, cwphysics_constraint **constraint, cwphysics_body *body_a, cwphysics_body *body_b, vec2 point_a, vec2 point_b, vec2 normal){
    int index = world->num_penetration_constraints;
    if(index >= PENETRATION_CONSTRAINTS_MAX_NUMBER) {
        return;
    }
    cwphysics_constraint_set_as_penetration(world->penetration_constraints + index, body_a, body_b, point_a, point_b, normal);
    world->num_penetration_constraints++;
    if(constraint == NULL) return;
    *constraint = world->penetration_constraints + index;
}

void cwphysics_world_update(cwphysics_world *world, float dt){
    for (int i = 0; i < BODIES_MAX_NUMBER; i++){
        cwphysics_body *body = world->bodies + i;
        if(!body->active) continue;
        vec2 weight = { 0.0f, world->gravity * body->mass }; //Mass cancels out
        cwphysics_body_add_force(body, weight);

        for (int i = 0; i < world->num_forces; i++){
            cwphysics_body_add_force(body, *(world->forces + i));
        }
        for (int i = 0; i < world->num_torques; i++){
            cwphysics_body_add_torque(body, *(world->torques + i));
        }

        cwphysics_body_integrate_forces(body, dt);
    }

    for (int i = 0; i < world->num_penetration_constraints; i++) cwphysics_constraint_free(world->penetration_constraints + i);

    world->num_contacts = 0;
    world->num_penetration_constraints = 0;
    for (int i = 0; i < BODIES_MAX_NUMBER; i++){
        cwphysics_body *body_a = world->bodies + i;
        for (int j = i + 1; j < BODIES_MAX_NUMBER; j++){
            cwphysics_body *body_b = world->bodies + j;
            if(!body_a->active || !body_b->active) continue;
            int contact_index = fminf(CONTACTS_MAX_NUMBER - 1, world->num_contacts);
            cwphysics_contact *contact = world->contacts + contact_index;
            if(cwphysics_collision_is_colliding(body_a, body_b, contact)){
                world->num_contacts++;
                cwphysics_world_add_and_fetch_penetration_constraint(world, NULL, contact->body_a, contact->body_b, contact->start, contact->end, contact->normal);
                // cwphysics_contact_resolve_collision(contact);
            }
        }
    }
    
    for (int i = 0; i < world->num_joint_constraints; i++){
        cwphysics_constraint_presolve(world->joint_constraints + i, dt);
    }
    for (int i = 0; i < world->num_penetration_constraints; i++){
        cwphysics_constraint_presolve(world->penetration_constraints + i, dt);
    }
    for (int x = 0; x < 5; x++){
        cwphysics_world_iterate_joint_constraints(world, cwphysics_constraint_solve);
        cwphysics_world_iterate_penetration_constraints(world, cwphysics_constraint_solve);
    }
    cwphysics_world_iterate_joint_constraints(world, cwphysics_constraint_postsolve);
    cwphysics_world_iterate_penetration_constraints(world, cwphysics_constraint_postsolve);

    for (int i = 0; i < BODIES_MAX_NUMBER; i++){
        cwphysics_body *body = world->bodies + i;
        if(!body->active) continue;
        cwphysics_body_integrate_velocities(body, dt);
        cwphysics_shape_update_vertices(body->shape, body->position, body->rotation);
    }
}

void cwphysics_world_free(cwphysics_world *world){
    for (int i = 0; i < BODIES_MAX_NUMBER; i++){
        cwphysics_body *body = world->bodies + i;
        if(!body->active) continue;
        cwphysics_body_clear(body);
    }
    free(world->bodies);

    for (int i = 0; i < world->num_joint_constraints; i++){
        cwphysics_constraint_free(world->joint_constraints + i);
    }
    free(world->joint_constraints);

    for (int i = 0; i < world->num_penetration_constraints; i++){
        cwphysics_constraint_free(world->penetration_constraints + i);
    }
    free(world->penetration_constraints);

    free(world->forces);
    free(world->torques);
    free(world->contacts);
    free(world);
}

void cwphysics_world_iterate_bodies(cwphysics_world *world, void (*func)(cwphysics_body*)){
    for (int i = 0; i < BODIES_MAX_NUMBER; i++){
        cwphysics_body *body = world->bodies + i;
        if(!body->active) continue;
        func(body);
    }
}

void cwphysics_world_iterate_contacts(cwphysics_world *world, void (*func)(cwphysics_contact*)){
    for (int i = 0; i < world->num_contacts; i++) func(world->contacts + i);
}

void cwphysics_world_iterate_joint_constraints(cwphysics_world *world, void (*func)(cwphysics_constraint*)){
    for (int i = 0; i < world->num_joint_constraints; i++) func(world->joint_constraints + i);
}

void cwphysics_world_iterate_penetration_constraints(cwphysics_world *world, void (*func)(cwphysics_constraint*)){
    for (int i = 0; i < world->num_penetration_constraints; i++) func(world->penetration_constraints + i);
}