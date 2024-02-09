#include "collision_detection.h"

bool cwphysics_collision_is_colliding(cwphysics_body *body_a, cwphysics_body *body_b, cwphysics_contact *contact){
    cwphysics_shape *shape_a = body_a->shape;
    cwphysics_shape *shape_b = body_b->shape;

    bool isCircle_a = shape_a->type == CIRCLE;
    bool isCircle_b = shape_b->type == CIRCLE;
    if(isCircle_a && isCircle_b){
        return cwphysics_collision_is_colliding_circle_circle(body_a, body_b, contact);
    }

    bool isPolygon_a = shape_a->type == POLYGON || shape_a->type == BOX;
    bool isPolygon_b = shape_b->type == POLYGON || shape_b->type == BOX;
    if(isPolygon_a && isPolygon_b){
        return cwphysics_collision_is_colliding_polygon_polygon(body_a, body_b, contact);
    }
    
    if(isCircle_a && isPolygon_b){
        return cwphysics_collision_is_colliding_circle_polygon(body_a, body_b, contact);
    }
    if(isCircle_b && isPolygon_a){
        return cwphysics_collision_is_colliding_circle_polygon(body_b, body_a, contact);
    }

    return false;
}

bool cwphysics_collision_is_colliding_circle_circle(cwphysics_body *body_a, cwphysics_body *body_b, cwphysics_contact *contact){
    cwphysics_shape_circle *circle_a = (cwphysics_shape_circle*)(body_a->shape->data);
    cwphysics_shape_circle *circle_b = (cwphysics_shape_circle*)(body_b->shape->data);
    vec2 ab;
    glm_vec2_sub(body_b->position, body_a->position, ab);
    float radiusSum = circle_a->radius + circle_b->radius;
    radiusSum *= radiusSum;
    float dist2 = glm_vec2_norm2(ab);
    bool isColliding = dist2 <= radiusSum;
    if(!isColliding) return false;

    contact->body_a = body_a;
    contact->body_b = body_b;
    glm_vec2_copy(ab, contact->normal);
    glm_vec2_normalize(contact->normal);

    vec2 normal_scale;
    glm_vec2_scale(contact->normal, circle_b->radius, normal_scale);
    glm_vec2_sub(body_b->position, normal_scale, contact->start);
    
    glm_vec2_scale(contact->normal, circle_a->radius, normal_scale);
    glm_vec2_add(body_a->position, normal_scale, contact->end);
    
    contact->depth = glm_vec2_distance(contact->end, contact->start);

    return true;
}

bool cwphysics_collision_is_colliding_polygon_polygon(cwphysics_body *body_a, cwphysics_body *body_b, cwphysics_contact *contact){
    cwphysics_shape_polygon *polygon_a = (cwphysics_shape_polygon*)(body_a->shape->data);
    cwphysics_shape_polygon *polygon_b = (cwphysics_shape_polygon*)(body_b->shape->data);

    vec2 normal_ab, point_ab;
    float separation_ab = cwphysics_shape_find_minimum_separation(polygon_a, polygon_b, &normal_ab, &point_ab);
    if(separation_ab >= 0.0f) return false;

    vec2 normal_ba, point_ba;
    float separation_ba = cwphysics_shape_find_minimum_separation(polygon_b, polygon_a, &normal_ba, &point_ba);
    if(separation_ba >= 0.0f) return false;
    
    float depth = 0.0f;
    vec2 normal;
    if(separation_ab > separation_ba){
        depth = -separation_ab;
        glm_vec2_copy(normal_ab, normal);
        glm_vec2_copy(point_ab, contact->start);
        glm_vec2_scale(normal_ab, depth, normal_ab);
        glm_vec2_add(point_ab, normal_ab, contact->end);
    }
    else{
        depth = -separation_ba;
        glm_vec2_copy(normal_ba, normal);
        glm_vec2_negate(normal);
        glm_vec2_copy(point_ba, contact->start);
        glm_vec2_scale(normal_ba, depth, normal_ba);
        glm_vec2_sub(point_ba, normal_ba, contact->end);
    }
    contact->depth = depth;
    glm_vec2_copy(normal, contact->normal);
    contact->body_a = body_a;
    contact->body_b = body_b;

    return true;
}

bool cwphysics_collision_is_colliding_circle_polygon(cwphysics_body *circle, cwphysics_body *polygon, cwphysics_contact *contact){
    cwphysics_shape_circle *circle_data = (cwphysics_shape_circle*)(circle->shape->data);
    cwphysics_shape_polygon *polygon_data = (cwphysics_shape_polygon*)(polygon->shape->data);
    
    // Find min distance edge
    float distance_circle_edge = -FLT_MAX;
    vec2 min_current_vertex, min_next_vertex, edge, min_edge;
    bool isOutside = false;
    for (int i = 0; i < polygon_data->numVertices; i++){
        vec2 circle_center;
        cwphysics_shape_polygon_edge_at(polygon_data, i, edge);
        vec2 normal = { -edge[1], edge[0] };
        glm_vec2_sub(circle->position, polygon_data->transformedVertices[i], circle_center);
        float projection = glm_vec2_dot(circle_center, normal);
        if(projection > 0 || projection > distance_circle_edge){
            glm_vec2_copy(edge, min_edge);
            distance_circle_edge = projection;
            glm_vec2_copy(polygon_data->transformedVertices[i], min_current_vertex);
            int next = (i + 1) % polygon_data->numVertices;
            glm_vec2_copy(polygon_data->transformedVertices[next], min_next_vertex);
        }
        if(projection > 0){
            isOutside = true;
            break;
        }
    }
    contact->body_a = circle;
    contact->body_b = polygon;

    //Projected point on edge (end position)
    vec2 current_to_circle;
    glm_vec2_sub(circle->position, min_current_vertex, current_to_circle);
    float t = glm_vec2_dot(min_edge, current_to_circle);
    t /= glm_vec2_norm2(min_edge);
    glm_vec2_lerp(min_current_vertex, min_next_vertex, t, contact->start);

    //Calculate normal
    vec2 start_to_circle;
    glm_vec2_sub(contact->start, circle->position, start_to_circle);
    if(isOutside){
        glm_vec2_sub(circle->position, contact->start, contact->normal);
    }
    else{
        glm_vec2_copy(start_to_circle, contact->normal);
    }
    glm_vec2_normalize(contact->normal);

    //Start point and direction
    vec2 normal_inv;
    glm_vec2_negate_to(contact->normal, normal_inv);
    glm_vec2_scale(normal_inv, circle_data->radius, contact->end);
    glm_vec2_add(contact->end, circle->position, contact->end);
    contact->depth = glm_vec2_distance(contact->end, contact->start);

    vec2 aux;
    glm_vec2_copy(contact->start, aux);
    glm_vec2_copy(contact->end, contact->start);
    glm_vec2_copy(aux, contact->end);

    if(isOutside){
        float dist2 = glm_vec2_norm2(start_to_circle);
        if(dist2 > circle_data->radius * circle_data->radius) return false;
    }

    return true;
}