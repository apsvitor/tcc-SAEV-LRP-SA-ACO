#include "vehicle.h"
#include <random>
#include <iostream>



Vehicle::Vehicle(int starting_point, int vehicle_id) {
    this->vehicle_path.push_back(starting_point);
    this->current_vertex    = starting_point;
    this->vehicle_id        = vehicle_id;

    this->current_battery   = vehicle_c::MAX_BATTERY;
    this->time_of_vehicle   = 0;
    this->is_recharging     = true;
}

