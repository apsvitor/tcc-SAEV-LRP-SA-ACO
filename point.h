#pragma once

class Point {
public:
    int     x;
    int     y;
    Point   ();
    Point   (int x, int y);
    double  get_distance(Point &b);
};