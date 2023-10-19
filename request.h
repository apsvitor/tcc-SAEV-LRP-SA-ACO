#pragma once
#include "vertex.h"

class Request : public Vertex{
public:
    Point   destination;
    int     pickup_time;
    double  request_distance;
    bool    is_done;
    bool    is_refused;
    int     closest_station;

    Request(Point origin, Point destination, int pickup_time, int request_id, int closest_station);
};
