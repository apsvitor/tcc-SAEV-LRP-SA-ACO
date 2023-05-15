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
    
    double                  __calculate_time_of_recharge();
    int                     __calculate_time_of_trip(Vertex *next_v);
    double                  __calculate_energy_of_trip(Vertex *next_v);
    double                  __calculate_distance_of_trip(Vertex *next_v);
    double                  __calculate_distance_of_request(Vertex *next_v);
    
    bool                    is_time_feasible(Vertex *next_v);
    bool                    is_energy_feasible(Vertex *next_v);
    void                    update_vehicle_state(Vertex *next_v);
    void                    update_vehicle_recharge(Vertex *next_v);
    void                    update_vehicle_request(Vertex *next_v);
};

/*
class Vehicle {
private:
    // temporary variables due to poor design
    double                  temp_time_spent;
    double                  temp_energy_spent;
    // Vehicle characteristics
    int                     vehicle_id;
    Point                   current_point;
    double                  current_battery;
    double                  time_of_vehicle;
    std::vector<Request>    request_list;

    std::vector<Vertex>     vehicle_path;
    

    double                  _calculate_time_of_trip(double distance);
    double                  _calculate_time_of_recharge();
    double                  _calculate_energy_of_trip(double distance);

public:
    Vehicle(Point starting_location, int vehicle_id);
    // ~Vehicle(){delete this;};
    int     get_vehicle_id();
    void    recharge_battery(double percentage=100.0);

    // Constraints
    
    bool    is_time_feasible(Request request);
    bool    is_time_feasible_with_recharge(Request request, Station recharge_station);
    bool    is_energy_feasible(Request request);
    bool    is_energy_feasible_with_recharge(Request request, Station recharge_station);

    // This method only works when both conditions are met.
    void    update_state_of_vehicle(
        bool time_feasibility, 
        bool energy_feasibility,
        Request new_request);

    void    update_state_of_recharged_vehicle(
        bool time_feasibility, 
        bool energy_feasibility,
        Request new_request);


    std::vector<Request> get_request_list();

};
*/