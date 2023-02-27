#pragma once

#include "point.h"

class Station {
private:
    Point   station_loc;
    int     station_id;
    int     free_spaces;
    bool    is_used;

public:
    Station();
    Station(Point location, int station_id, int free_spaces=3);

    int     get_station_id();
    Point   get_station_loc();
    int     get_free_spaces();
    bool    get_is_used();

    void    set_station_id(int station_id);
    void    set_station_loc(Point location);
    void    set_free_spaces(int value);
    void    set_is_used(bool value);

};