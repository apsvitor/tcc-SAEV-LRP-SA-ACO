#pragma once

#include "point.h"
class Request {
private:
    Point   origin;
    Point   destination;
    int     pickup_time;
    int     request_id;
    double  distance;

public:
    Request();
    Request(Point origin, Point destination, int pickup_time, int request_id);

    Point   get_origin();
    Point   get_destination();
    int     get_pickup_time();
    int     get_request_id();
    double  get_distance();

    void    set_origin(Point origin);
    void    set_destination(Point destination);
    void    set_pickup_time(int pickup_time);
    void    set_request_id(int request_id);
};