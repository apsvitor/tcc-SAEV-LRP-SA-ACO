#pragma once

#include <vector>
#include "point.h"

class Vertex {
public:
    Point                   p_xy;
    int                     vertex_id;
    char                    vertex_type;
    std::vector<Vertex*>    adj_list;
    
    Vertex(Point p, int vertex_id, char vertex_type);
};
