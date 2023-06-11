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
    std::vector<Vertex>     vehicle_path;
    Vertex                  *current_vertex;
    double                  current_battery;
    double                  time_of_vehicle;
    bool                    is_recharging;

    Vehicle(Vertex *starting_point, int vehicle_id);

    void                    add_vertex_to_vehicle_path(Vertex new_edge);    
    bool                    is_time_feasible(Vertex *next_v);
    bool                    is_energy_feasible(Vertex *next_v);
    void                    update_vehicle_request(Vertex *next_v);
};
