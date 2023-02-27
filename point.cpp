#include "point.h"
#include <cmath>

Point::Point() {}

Point::Point(int x, int y) {
    this->x = x;
    this->y = y;
}

int Point::get_x() {return this->x;};
int Point::get_y() {return this->y;};

double Point::get_distance(Point &b) {
        return sqrt((this->x - b.x)*(this->x - b.x) + (this->y - b.y)*(this->y - b.y));}
