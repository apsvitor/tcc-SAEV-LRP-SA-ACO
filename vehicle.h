#pragma once

#include <vector>
#include <queue>
#include "request.h"
#include "station.h"

class Vehicle {
private:
    // temporary variables due to poor design
    double                  temp_time_spent;
    double                  temp_energy_spent;
    // Vehicle FIXED characteristics
    int                     vehicle_id;
    double                  max_battery;        // KWh
    double                  consumption_rate;   // KWh per km
    double                  mean_velocity;      // km per min
    double                  charging_rate;      // Kwh per min
    double                  min_battery_level;  // percentage of charge

    Point                   current_point;
    double                  current_battery;
    double                     time_of_vehicle;
    std::vector<Request>    request_list;
    

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