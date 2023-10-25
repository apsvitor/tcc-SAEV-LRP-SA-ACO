#pragma once

#include <vector>
#include "point.h"

class Vertex {
public:
    Point                   p_xy;
    int                     vertex_id;
    char                    vertex_type;
    std::vector<int>        adj_list_int;
    
    Vertex(Point p, int vertex_id, char vertex_type);
};
