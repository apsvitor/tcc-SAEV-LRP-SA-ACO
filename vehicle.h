#pragma once

#include <vector>
#include <queue>
#include "request.h"
#include "station.h"
#include "vertex.h"
#include "constants.h"

class Vehicle {
public:
    int                     vehicle_id;
    std::vector<int>        vehicle_path;
    int                     current_vertex;
    double                  current_battery;
    double                  time_of_vehicle;
    bool                    is_recharging;

    Vehicle(int starting_point, int vehicle_id);

};
