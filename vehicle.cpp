#include "vehicle.h"
#include <random>
#include <iostream>

Vehicle::Vehicle(Vertex *starting_point, int vehicle_id) {
    this->current_vertex = starting_point;
    this->vehicle_id = vehicle_id;

    this->current_battery  = vehicle_c::MAX_BATTERY;
    this->time_of_vehicle   = 0;
}

void Vehicle::add_vertex_to_vehicle_path(Vertex new_edge) {
    this->vehicle_path.push_back(new_edge);
}

double Vehicle::__calculate_time_of_recharge() {
    return (vehicle_c::MAX_BATTERY - this->current_battery) / vehicle_c::CHARGING_RATE;
    
}

double Vehicle::__calculate_distance_of_trip(Vertex *next_v) {
    double total_distance = this->current_vertex->p_xy.get_distance(next_v->p_xy);
    if  (next_v->vertex_type == 'r')
        total_distance += static_cast<Request*>(next_v)->request_distance;
    return total_distance;
}

int Vehicle::__calculate_time_of_trip(Vertex* next_v) {
    double total_distance = __calculate_distance_of_trip(next_v);
    return std::ceil(total_distance / vehicle_c::MEAN_VELOCITY);
}

double Vehicle::__calculate_energy_of_trip(Vertex *next_v) {
    double total_distance = __calculate_distance_of_trip(next_v);
    return total_distance * vehicle_c::CONSUMPTION_RATE;
}

bool Vehicle::is_time_feasible(Vertex* next_v) {
    if  (this->current_vertex->vertex_type == 's')
        return true;
    int trip_time_cost = __calculate_time_of_trip(next_v);
    if  (this->time_of_vehicle + trip_time_cost <= 
        static_cast<Request*>(next_v)->pickup_time + request_c::LATENESS_EPS)
        return true;
    return false;
}

bool Vehicle::is_energy_feasible(Vertex *next_v) {
    double trip_energy_cost = __calculate_energy_of_trip(next_v);
    double min_energy_before_recharge = vehicle_c::MIN_BATTERY_LEVEL * vehicle_c::MAX_BATTERY;
    if  (this->current_battery - trip_energy_cost >= min_energy_before_recharge)
        return true;
    return false;
}

void Vehicle::update_vehicle_recharge(Vertex *next_v) {
    double energy_cost          = __calculate_energy_of_trip(next_v);
    this->current_battery       -= energy_cost;
    double trip_time_cost       = __calculate_time_of_trip(next_v);
    double recharge_time_cost   = __calculate_time_of_recharge();
    
    // for now it's 100%. Adapt to flexible percentage later.
    this->current_battery = vehicle_c::MAX_BATTERY;
    this->time_of_vehicle += trip_time_cost + recharge_time_cost;
}

void Vehicle::update_vehicle_request(Vertex *next_v) {
    double energy_cost          = __calculate_energy_of_trip(next_v);
    this->current_battery       -= energy_cost;
    double trip_time_cost       = __calculate_time_of_trip(next_v);
    
    // for now it's 100%. Adapt to flexible percentage later.
    this->current_battery = vehicle_c::MAX_BATTERY;
    this->time_of_vehicle += trip_time_cost;
}

// old implementation - no graph
/*
Vehicle::Vehicle(Point starting_location, int vehicle_id) {
    this->vehicle_id = vehicle_id;
    
    this->current_battery   = vehicle_c::MAX_BATTERY;
    this->time_of_vehicle   = 0;
    this->current_point     = starting_location;

    this->temp_energy_spent = 0;
    this->temp_time_spent   = 0;
}

void Vehicle::recharge_battery(double percentage) {
    this->current_battery = vehicle_c::MAX_BATTERY*percentage;
}

int Vehicle::get_vehicle_id(){return this->vehicle_id;}

bool Vehicle::is_time_feasible(Request request) {
    // Can this vehicle perform a given Request in time?
    Point origin_aux    = request.get_origin();
    double origin_dist  = this->current_point.get_distance(origin_aux);
    double trip_dist    = request.get_distance();
    int total_time      = this->_calculate_time_of_trip(origin_dist+trip_dist);
    if  (total_time + this->time_of_vehicle > request.get_pickup_time())
        return false;
    this->temp_time_spent = total_time; 
    return true;
}

bool Vehicle::is_time_feasible_with_recharge(Request request, Station recharge_station) {
    // Can this vehicle recharge and perform a given Request in time?
    Point station_aux   = recharge_station.get_station_loc();
    Point origin_aux    = request.get_origin();
    
    double recharge_dist= this->current_point.get_distance(station_aux);
    double origin_dist  = station_aux.get_distance(origin_aux);
    double trip_dist    = request.get_distance();
    
    double total_time      = _calculate_time_of_recharge();
    total_time          += this->_calculate_time_of_trip(recharge_dist + origin_dist + trip_dist);
    if  (total_time + this->time_of_vehicle > request.get_pickup_time())
        return false;
    this->temp_time_spent = total_time; 
    return true;
}


bool Vehicle::is_energy_feasible(Request request) {
    // Can this vehicle perform a given Request with its remaining battery?
    Point origin_aux    = request.get_origin();
    double origin_dist  = this->current_point.get_distance(origin_aux);
    double trip_dist    = request.get_distance();
    double total_energy    = this->_calculate_energy_of_trip(origin_dist+trip_dist);
    if  (this->current_battery - total_energy < vehicle_c::MAX_BATTERY * vehicle_c::MIN_BATTERY_LEVEL)
        return false;
    this->temp_energy_spent = total_energy;
    return true;
}

bool Vehicle:: is_energy_feasible_with_recharge(Request request, Station recharge_station) {
    // Can this vehicle recharge and perform a given Request in time?
    Point station_aux   = recharge_station.get_station_loc();
    Point origin_aux    = request.get_origin();
    
    double recharge_dist= this->current_point.get_distance(station_aux);
    double total_energy = this->_calculate_energy_of_trip(recharge_dist);
    std::cout << "Energy to station: " << total_energy << "| Energy left: " << this->current_battery << std::endl;
    // check if this vehicle can reach the station
    if  (total_energy > this->current_battery)
        return false;
    
    double origin_dist  = station_aux.get_distance(origin_aux);
    double trip_dist    = request.get_distance();
    
    total_energy        = this->_calculate_energy_of_trip(origin_dist+trip_dist);
    if  (this->current_battery - total_energy < vehicle_c::MAX_BATTERY * vehicle_c::MIN_BATTERY_LEVEL)
        return false;
    std::cout << "After recharge energy spent: " << total_energy << std::endl;
    this->temp_energy_spent = total_energy;
    return true;
}

double Vehicle::_calculate_time_of_trip(double distance) {
    // returns time in minutes (rounded up) 
    return distance/vehicle_c::MEAN_VELOCITY;
}

double Vehicle::_calculate_time_of_recharge() {
    // returns time in minutes (rounded up)
    return (vehicle_c::MAX_BATTERY - this->current_battery) / vehicle_c::CHARGING_RATE;
}

double Vehicle::_calculate_energy_of_trip(double distance) {
    // returns energy to perform the trip
    return distance * vehicle_c::CONSUMPTION_RATE;
}

void Vehicle::update_state_of_vehicle(
    bool time_feasibility, 
    bool energy_feasibility,
    Request new_request) {
    if  (time_feasibility && energy_feasibility) {
        this->current_battery   -= this->temp_energy_spent;
        this->time_of_vehicle   += this->temp_time_spent;
        this->current_point      = new_request.get_destination();
        this->request_list.push_back(new_request);
    }
}

void Vehicle::update_state_of_recharged_vehicle(
    bool time_feasibility, 
    bool energy_feasibility,
    Request new_request) {
    if  (time_feasibility && energy_feasibility) {
        Request trip_to_station = Request(this->current_point, new_request.get_origin(), 0, 99);
        this->request_list.push_back(trip_to_station);
        this->current_battery   = vehicle_c::MAX_BATTERY - this->temp_energy_spent;
        this->time_of_vehicle   += this->temp_time_spent;
        this->current_point     = new_request.get_destination();
        this->request_list.push_back(new_request);
    }
}


std::vector<Request> Vehicle::get_request_list() {return this->request_list;}

*/