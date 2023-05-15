#include "vehicle.h"
#include <random>
#include <iostream>

Vehicle::Vehicle(Vertex *starting_point, int vehicle_id) {
    this->current_vertex    = starting_point;
    this->vehicle_id        = vehicle_id;

    this->current_battery   = vehicle_c::MAX_BATTERY;
    this->time_of_vehicle   = 0;
    this->is_recharging     = true;
}

void Vehicle::add_vertex_to_vehicle_path(Vertex new_edge) {
    this->vehicle_path.push_back(new_edge);
}

// double Vehicle::__calculate_time_of_recharge() {
//     return (vehicle_c::MAX_BATTERY - this->current_battery) / vehicle_c::CHARGING_RATE;
// }

// double Vehicle::__calculate_distance_of_trip(Vertex *next_v) {
//     double total_distance = this->current_vertex->p_xy.get_distance(next_v->p_xy);
//     // if  (next_v->vertex_type == 'r')
//         // total_distance += static_cast<Request*>(next_v)->request_distance;
//     return total_distance;
// }

// double Vehicle::__calculate_distance_of_request(Vertex *next_v) {
//     return static_cast<Request*>(next_v)->request_distance;
// }

// int Vehicle::__calculate_time_of_trip(Vertex* next_v) {
//     double total_distance = __calculate_distance_of_trip(next_v);
//     return std::ceil(total_distance / vehicle_c::MEAN_VELOCITY);
// }

// double Vehicle::__calculate_energy_of_trip(Vertex *next_v) {
//     double total_distance = __calculate_distance_of_trip(next_v);
//     return total_distance * vehicle_c::CONSUMPTION_RATE;
// }

bool Vehicle::is_time_feasible(Vertex* next_v) {
    // can the vehicle reach the origin on time? ----------------------------------------
    if  (next_v->vertex_type == 's')
        return true;
    // if the vehicle was previously at a station, the recharge time will be
    // taken into account to check whether it is possible to partially recharge
    // before answering a request
    
    if  (this->is_recharging) {
        double distance_to_origin   = this->current_vertex->p_xy.get_distance(next_v->p_xy),
               distance_of_request  = static_cast<Request*>(next_v)->request_distance,
               total_distance       = distance_to_origin + distance_of_request;

        double vehicle_energy   = this->current_battery,
               min_energy       = vehicle_c::MIN_BATTERY_LEVEL * vehicle_c::MAX_BATTERY,
               request_energy   = total_distance * vehicle_c::CONSUMPTION_RATE,
               min_recharge     = (min_energy + request_energy) - vehicle_energy;
        
        int vehicle_time        = this->time_of_vehicle,
            reach_origin_time   = std::ceil(distance_to_origin / vehicle_c::MEAN_VELOCITY),
            pickup_time         = static_cast<Request*>(next_v)->pickup_time;

        if  (problem_type::IS_PARTIAL_RECHARGE && min_recharge > 0) {
            // tempo pra carregar o minimo necessario pra fazer a request
            // implicitamente checa energy_feasibility
            int min_recharge_time   = std::ceil(min_recharge / vehicle_c::CHARGING_RATE),
                request_start_time  = vehicle_time + reach_origin_time + min_recharge_time;
            // o tempo para recarregar o minimo está no intervalo de atraso permitido
            if  (request_start_time >= pickup_time && 
                 request_start_time + request_c::LATENESS_EPS <= pickup_time){
                // its possible to partially recharge
                this->current_battery += min_recharge;
                this->time_of_vehicle = request_start_time;
                this->is_recharging = false;
                return true;
            }
            return false;
        }
        else if (!problem_type::IS_PARTIAL_RECHARGE) {
            // full recharge
            double energy_missing = vehicle_c::MAX_BATTERY - this->current_battery;
            int time_to_recharge = std::ceil(energy_missing / vehicle_c::CHARGING_RATE);

            this->time_of_vehicle += time_to_recharge;
            this->current_battery += energy_missing;
            this->is_recharging = false;
        }
        if  (this->time_of_vehicle + reach_origin_time <= pickup_time + request_c::LATENESS_EPS) {
            this->is_recharging = false;
            return true;
        }
        return false;
    }

    double distance_to_request = this->current_vertex->p_xy.get_distance(next_v->p_xy);
    int start_request_time = std::ceil(distance_to_request / vehicle_c::MEAN_VELOCITY);
    if  (this->time_of_vehicle + start_request_time <= 
         static_cast<Request*>(next_v)->pickup_time + request_c::LATENESS_EPS)
        return true;
    return false;
    // TEST END -------------------------------------------------------------------------
    
    // if  (next_v->vertex_type == 's')
    //     return true;
    // int trip_time_cost = __calculate_time_of_trip(next_v);
    // if  (this->time_of_vehicle + trip_time_cost <= 
    //     static_cast<Request*>(next_v)->pickup_time + request_c::LATENESS_EPS)
    //     return true;
    // return false;
}

bool Vehicle::is_energy_feasible(Vertex *next_v) {
    // can the vehicle do the whole trip? -----------------------------------------------
    double total_distance = this->current_vertex->p_xy.get_distance(next_v->p_xy);
    if  (next_v->vertex_type == 'r')
        total_distance += (static_cast<Request*>(next_v)->request_distance);
    double total_energy_cost = total_distance * vehicle_c::CONSUMPTION_RATE;
    double min_energy_before_recharge = vehicle_c::MIN_BATTERY_LEVEL * vehicle_c::MAX_BATTERY;
    if  (this->current_battery - total_energy_cost >= min_energy_before_recharge)
        return true;
    return false;
    // TEST END -------------------------------------------------------------------------
    
    // double trip_energy_cost = __calculate_energy_of_trip(next_v);
    // double min_energy_before_recharge = vehicle_c::MIN_BATTERY_LEVEL * vehicle_c::MAX_BATTERY;
    // if  (this->current_battery - trip_energy_cost >= min_energy_before_recharge)
    //     return true;
    // return false;
}

// void Vehicle::update_vehicle_recharge(Vertex *next_v) {
//     double energy_cost          = __calculate_energy_of_trip(next_v);
//     this->current_battery       -= energy_cost;
//     double trip_time_cost       = __calculate_time_of_trip(next_v);
//     double recharge_time_cost   = __calculate_time_of_recharge();
    
//     // for now it's 100%. Adapt to flexible percentage later.
//     this->current_battery = vehicle_c::MAX_BATTERY;
//     this->time_of_vehicle += trip_time_cost + recharge_time_cost;
// }

void Vehicle::update_vehicle_request(Vertex *next_v) {
    // updates vehicle state after it answers a request ---------------------------------
    this->is_recharging = false;
    double total_distance = this->current_vertex->p_xy.get_distance(next_v->p_xy);
    if  (next_v->vertex_type == 'r')
        total_distance += (static_cast<Request*>(next_v)->request_distance);
    else
        this->is_recharging = true;
    
    double energy_cost = total_distance * vehicle_c::CONSUMPTION_RATE;
    int time_cost = std::ceil(total_distance / vehicle_c::MEAN_VELOCITY);

    this->current_battery -= energy_cost;
    this->time_of_vehicle += time_cost;

    // TEST END -------------------------------------------------------------------------

    // double energy_cost          = __calculate_energy_of_trip(next_v);
    // this->current_battery       -= energy_cost;
    // double trip_time_cost       = __calculate_time_of_trip(next_v);
    

    // // This is only used when Partial Recharge is off
    // // for now it's 100%. Adapt to flexible percentage later.
    // // ERRO GRAVE
    // this->current_battery = vehicle_c::MAX_BATTERY;
    
    // this->time_of_vehicle += trip_time_cost;
}
