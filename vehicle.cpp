#include "vehicle.h"
#include <cmath>
#include <random>
#include <queue>
#include <iostream>


Vehicle::Vehicle(Point starting_location, int vehicle_id) {
    this->vehicle_id = vehicle_id;
    this->max_battery       = 15.0;
    this->consumption_rate  = 0.15;
    this->mean_velocity     = 35.0/60;
    this->charging_rate     = 0.4;
    this->min_battery_level = 0.20;

    this->current_battery   = this->max_battery;
    this->time_of_vehicle   = 0;
    this->current_point     = starting_location;

    this->temp_energy_spent = 0;
    this->temp_time_spent   = 0;
}

void Vehicle::recharge_battery(double percentage) {
    this->current_battery = this->max_battery*percentage;
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
    if  (this->current_battery - total_energy < this->max_battery*this->min_battery_level)
        return false;
    this->temp_energy_spent = total_energy;
    return true;
}

bool Vehicle:: is_energy_feasible_with_recharge(Request request, Station recharge_station) {
    // Can this vehicle recharge and perform a given Request in time?
    Point station_aux   = recharge_station.get_station_loc();
    Point origin_aux    = request.get_origin();
    
    double recharge_dist= this->current_point.get_distance(station_aux);
    double total_energy    = this->_calculate_energy_of_trip(recharge_dist);
    std::cout << "Energy to station: " << total_energy << "| Energy left: " << this->current_battery << std::endl;
    // check if this vehicle can reach the station
    if  (total_energy > this->current_battery)
        return false;
    
    double origin_dist  = station_aux.get_distance(origin_aux);
    double trip_dist    = request.get_distance();
    
    total_energy        = this->_calculate_energy_of_trip(origin_dist+trip_dist);
    if  (this->current_battery - total_energy < this->max_battery*this->min_battery_level)
        return false;
    std::cout << "After recharge energy spent: " << total_energy << std::endl;
    this->temp_energy_spent = total_energy;
    return true;
}

double Vehicle::_calculate_time_of_trip(double distance) {
    // returns time in minutes (rounded up) 
    return std::ceil(distance/this->mean_velocity);
}

double Vehicle::_calculate_time_of_recharge() {
    // returns time in minutes (rounded up)
    return std::ceil((this->max_battery - this->current_battery) / this->charging_rate);
}

double Vehicle::_calculate_energy_of_trip(double distance) {
    // returns energy to perform the trip
    return distance*this->consumption_rate;
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
        this->current_battery   = this->max_battery - this->temp_energy_spent;
        this->time_of_vehicle   += this->temp_time_spent;
        this->current_point     = new_request.get_destination();
        this->request_list.push_back(new_request);
    }
}


std::vector<Request> Vehicle::get_request_list() {return this->request_list;}