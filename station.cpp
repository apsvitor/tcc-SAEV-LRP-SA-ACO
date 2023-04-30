#include "station.h"
Station::Station(Point station_xy, int station_id)
        : Vertex(station_xy, station_id, 's') {
    this->is_used = 0;
    this->free_spaces = 5; // unused for now
}
