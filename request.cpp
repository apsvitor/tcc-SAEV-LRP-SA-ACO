#include "request.h"

Request::Request() {}

Request::Request(Point origin, Point destination, int pickup_time, int request_id) {
    this->origin = origin;
    this->destination = destination;
    this->pickup_time = pickup_time;
    this->request_id = request_id;
    this->distance = origin.get_distance(destination);
}

Point Request::get_origin() {return this->origin;}
void Request::set_origin(Point origin) {this->origin = origin;}

Point Request::get_destination() {return this->destination;}
void Request::set_destination(Point destination) {this->destination = destination;}

int Request::get_pickup_time() {return this->pickup_time;}
void Request::set_pickup_time(int pickup_time) {this->pickup_time = pickup_time;}

int Request::get_request_id() {return this->request_id;}
void Request::set_request_id(int request_id) {this->request_id = request_id;}

double Request::get_distance() {return this->distance;}
