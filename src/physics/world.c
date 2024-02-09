#include "world.h"

void cwphysics_world_get(cwphysics_world **world, float gravity){
    cwphysics_world *aux = malloc(sizeof(cwphysics_world));
    aux->gravity = gravity;
    aux->num_forces = 0;
    aux->num_torques = 0;
    aux->num_contacts = 0;
    aux->num_constraints = 0;

    aux->bodies = (cwphysics_body*)malloc(sizeof(cwphysics_body) * BODIES_MAX_NUMBER);
    aux->forces = (vec2*)malloc(sizeof(vec2) * FORCES_MAX_NUMBER);
    aux->torques = (float*)malloc(sizeof(float) * TORQUES_MAX_NUMBER);
    aux->contacts = (cwphysics_contact*)malloc(sizeof(cwphysics_contact) * CONTACTS_MAX_NUMBER);
    aux->constraints = (cwphysics_constraint*)malloc(sizeof(cwphysics_constraint) * CONSTRAINTS_MAX_NUMBER);

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
    if(index > FORCES_MAX_NUMBER) return;
    glm_vec2_copy(force, *(world->forces + index));
    world->num_forces++;
}

void cwphysics_world_add_torque(cwphysics_world *world, float torque){
    int index = world->num_torques;
    if(index > TORQUES_MAX_NUMBER) return;
    *(world->torques + index) = torque;
    world->num_torques++;
}

void cwphysics_world_add_and_fetch_constraint(cwphysics_world *world, cwphysics_constraint **constraint){
    int index = world->num_constraints;
    if(index > CONSTRAINTS_MAX_NUMBER) {
        return;
    }
    *constraint = world->constraints + index;
    (*(*constraint)).type = NO_CONSTRAINT;
    (*(*constraint)).bias = 0.0f;
    world->num_constraints++;
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

    int num_penetrations = 0;
    cwphysics_constraint *penetrations = (cwphysics_constraint*)malloc(sizeof(cwphysics_constraint) * PENETRATIONS_MAX_NUMBER);

    world->num_contacts = 0;
    for (int i = 0; i < BODIES_MAX_NUMBER; i++){
        cwphysics_body *body_a = world->bodies + i;
        for (int j = i + 1; j < BODIES_MAX_NUMBER; j++){
            cwphysics_body *body_b = world->bodies + j;
            if(!body_a->active || !body_b->active) continue;
            int contact_index = fminf(CONTACTS_MAX_NUMBER - 1, world->num_contacts);
            cwphysics_contact *contact = world->contacts + contact_index;
            if(cwphysics_collision_is_colliding(body_a, body_b, contact)){
                // cwphysics_contact_resolve_collision(contact);
                world->num_contacts++;
                if(num_penetrations >= PENETRATIONS_MAX_NUMBER) continue;
                cwphysics_constraint *penetration = penetrations + num_penetrations;
                cwphysics_constraint_set_as_penetration(penetration, contact->body_a, contact->body_b, contact->start, contact->end, contact->normal);
                num_penetrations++;
            }
        }
    }
    
    for (int i = 0; i < world->num_constraints; i++){
        cwphysics_constraint_presolve(world->constraints + i, dt);
    }
    for (int i = 0; i < num_penetrations; i++){
        cwphysics_constraint_presolve(penetrations + i, dt);
    }
    for (int x = 0; x < 5; x++){
        cwphysics_world_iterate_constraints(world, cwphysics_constraint_solve);
        for (int i = 0; i < num_penetrations; i++){
            cwphysics_constraint_solve(penetrations + i);
        }
    }
    cwphysics_world_iterate_constraints(world, cwphysics_constraint_postsolve);
    for (int i = 0; i < num_penetrations; i++){
        cwphysics_constraint_postsolve(penetrations + i);
    }

    free(penetrations);

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
    for (int i = 0; i < world->num_constraints; i++){
        cwphysics_constraint *constraint = world->constraints + i;
        cwphysics_constraint_free(constraint);
    }
    free(world->constraints);

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

void cwphysics_world_iterate_constraints(cwphysics_world *world, void (*func)(cwphysics_constraint*)){
    for (int i = 0; i < world->num_constraints; i++) func(world->constraints + i);
}