#ifndef _SHAPE_H
#define _SHAPE_H

#include <cglm/cglm.h>
#include <stdbool.h>

#define SHAPE_MAX_NUMBER 100

typedef enum{
    CIRCLE,
    POLYGON,
    BOX
} cwphysics_shape_type;

typedef struct{
    void *data;
    cwphysics_shape_type type;
    bool active;
    char pad[3];
} cwphysics_shape;

typedef struct{
    float radius;
} cwphysics_shape_circle;

typedef struct{
    vec2* vertices;
    vec2* transformedVertices;
    vec2 scale;
    int numVertices;
} cwphysics_shape_polygon;

typedef struct{
    cwphysics_shape_polygon polygon;
    float width;
    float height;
} cwphysics_shape_box;

bool cwphysics_shape_get_circle(cwphysics_shape **shape, float radius);
bool cwphysics_shape_get_polygon(cwphysics_shape **shape, vec2 *vertices, int numVertices, float scale);
bool cwphysics_shape_get_box(cwphysics_shape **shape, float width, float height);
void cwphysics_shape_clear(cwphysics_shape *shape);
float cwphysics_shape_get_moment_inertia(cwphysics_shape *shape);
void cwphysics_shape_update_vertices(cwphysics_shape *shape, vec2 position, float rotation);
void cwphysics_shape_polygon_edge_at(cwphysics_shape_polygon *polygon, int vertex_index, vec2 edge);
void cwphysics_shape_polygon_edge_at_inv(cwphysics_shape_polygon *polygon, int vertex_index, vec2 edge);
float cwphysics_shape_find_minimum_separation(cwphysics_shape_polygon *shape_a, cwphysics_shape_polygon *shape_b, vec2 *normal, vec2 *point);

#endif