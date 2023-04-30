#pragma once

#include "vertex.h"

class Station : public Vertex{
public:
    int     free_spaces;
    int     is_used;

    Station(Point station_xy, int station_id);
};
