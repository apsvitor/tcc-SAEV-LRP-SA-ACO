#include "point.h"
#include <cmath>

Point::Point() {}

Point::Point(double x, double y) {
    this->x = x;
    this->y = y;
}

double Point::get_distance(Point &b) {
        return sqrt((this->x - b.x)*(this->x - b.x) + (this->y - b.y)*(this->y - b.y));}
