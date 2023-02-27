#pragma once

class Point {
public:
    int     x;
    int     y;

    Point   ();
    Point   (int x, int y);
    int     get_x();
    int     get_y();
    double  get_distance(Point &b);
};