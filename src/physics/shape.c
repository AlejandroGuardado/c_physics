#include "shape.h"

static cwphysics_shape shapes[SHAPE_MAX_NUMBER];
static cwphysics_shape_circle circle_data[SHAPE_MAX_NUMBER];
static cwphysics_shape_polygon polygon_data[SHAPE_MAX_NUMBER];
static cwphysics_shape_box box_data[SHAPE_MAX_NUMBER];

bool cwphysics_shape_get_circle(cwphysics_shape **shape, float radius){
    for (int i = 0; i < SHAPE_MAX_NUMBER; i++){
        if(shapes[i].active) continue;
        circle_data[i].radius = radius;
        shapes[i].type = CIRCLE;
        shapes[i].data = &circle_data[i];
        shapes[i].active = true;
        *shape = &shapes[i];
        return true;
    }
    return false;
}

bool cwphysics_shape_get_polygon(cwphysics_shape **shape, vec2 *vertices, int numVertices, float scale){
    for (int i = 0; i < SHAPE_MAX_NUMBER; i++){
        if(shapes[i].active) continue;
        polygon_data[i].vertices = vertices;
        polygon_data[i].numVertices = numVertices;
        shapes[i].type = POLYGON;

        polygon_data[i].transformedVertices = malloc(sizeof(vec2) * numVertices);
        vec2 *transformedVertices = polygon_data[i].transformedVertices;
        if(transformedVertices == NULL) return false;
        for (int i = 0; i < numVertices; i++){
            glm_vec2_scale(*(vertices + i), scale, *(vertices + i));
            glm_vec2_copy(*(vertices + i), *(transformedVertices + i));
        }
        vec2 scale_v = { scale, scale };
        glm_vec2_copy(scale_v, polygon_data[i].scale);
        shapes[i].data = &polygon_data[i];
        shapes[i].active = true;
        *shape = &shapes[i];
        return true;
    }
    return false;
}

bool cwphysics_shape_get_box(cwphysics_shape **shape, float width, float height){
    for (int i = 0; i < SHAPE_MAX_NUMBER; i++){
        if(shapes[i].active) continue;
        box_data[i].width = width;
        box_data[i].height = height;

        box_data[i].polygon.vertices = malloc(sizeof(vec2) * 4);
        box_data[i].polygon.transformedVertices = malloc(sizeof(vec2) * 4);
        vec2 *vertices, *transformedVertices;
        vertices = box_data[i].polygon.vertices;
        transformedVertices = box_data[i].polygon.transformedVertices;

        if(vertices == NULL || transformedVertices == NULL) return false;
        float w = width * 0.5f;
        float h = height * 0.5f;
        (*vertices)[0] = -w;  
        (*vertices)[1] = h;
        (*(vertices + 1))[0] = w;  
        (*(vertices + 1))[1] = h;
        (*(vertices + 2))[0] = w;  
        (*(vertices + 2))[1] = -h;
        (*(vertices + 3))[0] = -w;  
        (*(vertices + 3))[1] = -h;
        box_data[i].polygon.vertices = vertices;
        for (int i = 0; i < 4; i++){
            glm_vec2_copy(*(vertices + i), *(transformedVertices + i));
        }
        box_data[i].polygon.transformedVertices = transformedVertices;
        box_data[i].polygon.numVertices = 4;

        shapes[i].type = BOX;
        shapes[i].data = &box_data[i];
        vec2 scale_v = { width, height };
        glm_vec2_copy(scale_v, box_data[i].polygon.scale);
        shapes[i].active = true;
        *shape = &shapes[i];
        return true;
    }
    return false;
}

void cwphysics_shape_clear(cwphysics_shape *shape){
    if(shape == NULL || !shape->active) return;
    shape->active = false;
    switch (shape->type){
        case POLYGON:
            free(((cwphysics_shape_polygon*)shape->data)->vertices);
            free(((cwphysics_shape_polygon*)shape->data)->transformedVertices);
            break;
        case BOX:
            free(((cwphysics_shape_box*)(shape->data))->polygon.vertices);
            free(((cwphysics_shape_box*)shape->data)->polygon.transformedVertices);
            break;
        case CIRCLE:
        default:
            break;
    }
}

float cwphysics_shape_get_moment_inertia(cwphysics_shape *shape){
    switch (shape->type){
        case CIRCLE:
            cwphysics_shape_circle *circle_data = (cwphysics_shape_circle*)(shape->data);
            float circle_inertia = circle_data->radius * circle_data->radius;
            return circle_inertia * 0.5f;
        case BOX:
            cwphysics_shape_box *box_data = (cwphysics_shape_box*)(shape->data);
            float box_inertia = box_data->width * box_data->width + box_data->height * box_data->height;
            return box_inertia * 0.083333f; // 1/12
        case POLYGON:
            return 1.0f; //TODO
        default:
            return 0.0f;
    }
}

void cwphysics_shape_update_vertices(cwphysics_shape *shape, vec2 position, float rotation){
    vec2 *vertices;
    vec2 *transformedVertices;
    int numVertices;
    switch (shape->type){
        case POLYGON:
        case BOX:
            cwphysics_shape_polygon *polygon_data = (cwphysics_shape_polygon*)(shape->data);
            vertices = polygon_data->vertices;
            transformedVertices = polygon_data->transformedVertices;
            numVertices = polygon_data->numVertices;
            break;
        case CIRCLE:
        default:
            return;
    }

    for (int i = 0; i < numVertices; i++){
        glm_vec2_rotate(*(vertices + i), rotation, *(transformedVertices + i));
        glm_vec2_add(*(transformedVertices + i), position, *(transformedVertices + i));
    }
}

void cwphysics_shape_polygon_edge_at(cwphysics_shape_polygon *polygon, int vertex_index, vec2 edge){
    glm_vec2_zero(edge);
    if(vertex_index < 0 || vertex_index >= polygon->numVertices) return;
    vec2 current;
    glm_vec2_copy(*(polygon->transformedVertices + vertex_index), current);
    vec2 next;
    int next_index = vertex_index + 1;
    next_index %= polygon->numVertices;
    glm_vec2_copy(*(polygon->transformedVertices + next_index), next);
    glm_vec2_sub(next, current, edge);
}

void cwphysics_shape_polygon_edge_at_inv(cwphysics_shape_polygon *polygon, int vertex_index, vec2 edge){
    glm_vec2_zero(edge);
    if(vertex_index < 0 || vertex_index >= polygon->numVertices) return;
    vec2 current;
    glm_vec2_copy(*(polygon->transformedVertices + vertex_index), current);
    vec2 next;
    int next_index = vertex_index + 1;
    next_index %= polygon->numVertices;
    glm_vec2_copy(*(polygon->transformedVertices + next_index), next);
    glm_vec2_sub(current, next, edge);
}

float cwphysics_shape_find_minimum_separation(cwphysics_shape_polygon *shape_a, cwphysics_shape_polygon *shape_b, vec2 *normal, vec2 *point){
    float separation = -FLT_MAX;
    for (int index_a = 0; index_a < shape_a->numVertices; index_a++){
        vec2 vertex_a;
        glm_vec2_copy(*(shape_a->transformedVertices + index_a), vertex_a);
        vec2 edge;
        cwphysics_shape_polygon_edge_at(shape_a, index_a, edge);
        vec2 edge_normal = { -edge[1], edge[0] }; //perpendicular
        glm_vec2_normalize(edge_normal);

        float minimum_separation = FLT_MAX;
        vec2 min_vertex;
        for (int index_b = 0; index_b < shape_b->numVertices; index_b++){
            vec2 vertex_b;
            glm_vec2_copy(*(shape_b->transformedVertices + index_b), vertex_b);
        
            vec2 dir;
            glm_vec2_sub(vertex_b, vertex_a, dir);
            float projection = glm_vec2_dot(dir, edge_normal);
            if(projection < minimum_separation){
                minimum_separation = projection;
                glm_vec2_copy(vertex_b, min_vertex);
            }
        }
        if(separation < minimum_separation){
            separation = minimum_separation;
            glm_vec2_copy(edge_normal, *normal);
            glm_vec2_copy(min_vertex, *point);
        }
    }
    return separation;
}