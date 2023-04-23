#include "request.h"

Request::Request(Point origin, Point destination, int pickup_time, int request_id)
    : Vertex(origin, request_id, 'r') {
    this->destination = destination;
    this->pickup_time = pickup_time;
    this->request_distance = origin.get_distance(destination);
    this->is_done = false;
}
