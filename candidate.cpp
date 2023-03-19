#include "candidate.h"
#include <random>
#include <iostream>

Candidate::Candidate(std::queue<Request> all_requests,
                     std::vector<Station> all_stations) {
    this->all_requests = all_requests;
    this->all_stations = all_stations;
}

void Candidate::generate_candidate() {
    // auxiliary part
    int num_of_stations = this->all_stations.size();
    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());
    std::uniform_int_distribution<int> rand_station(0, num_of_stations-1);
    
    int         v_index = -1;            // which vehicle is receiving requests
    int         req_index = 0;          // which request is being used
    bool        acquire_vehicle = true;
    Vehicle*    car_pointer;
    // It is necessary to allocate ALL requests
    while(!this->all_requests.empty()){
        Request request = this->all_requests.front();
        // verify if a new vehicle is needed
        Point o = request.get_origin();
        Point d = request.get_destination();
        int   t = request.get_pickup_time();
        std::cout << "CLIENT LOOKING FOR VEHICLE: O(" << o.get_x() << ", " << o.get_y() << ") | D(" 
                      << d.get_x() << ", " << d.get_y() << ") | T = " << t << std::endl;
        if  (acquire_vehicle){
            int pos = rand_station(generator);
            Point station_loc = this->all_stations[pos].get_station_loc();
            this->all_stations[pos].set_is_used(true);
            std::cout << "Vehicle acquired: V=" << v_index+1 <<"\t\tLeaving Station at: (" << station_loc.x << ", " << station_loc.y << ")\n";
            car_pointer = new Vehicle(station_loc, ++v_index);
            this->all_vehicles.push_back(*car_pointer);
            acquire_vehicle = false;
        }
        // Validating conditions for a given vehicle accepting a request
        bool has_time = this->all_vehicles[v_index].is_time_feasible(request);
        // std::cout << "has_time: " << has_time << std::endl;
        if  (has_time) {
            // can complete request in time
            bool has_energy = this->all_vehicles[v_index].is_energy_feasible(request);
            // std::cout << "has_energy: " << has_energy << std::endl;
            if  (has_energy) {
                // has energy to complete the request
                std::cout   << "\tVehicle: " << car_pointer->get_vehicle_id() << " updated, added " 
                            << "request: O(" << o.get_x() << ", " << o.get_y() << ") | D(" 
                            << d.get_x() << ", " << d.get_y() << ") | T = " << t << std::endl;
                this->all_vehicles[v_index].update_state_of_vehicle(has_time, has_energy, request);
                this->all_requests.pop();
            }
            else {
                // does not have energy to complete the request. Recharge needed
                int pos = rand_station(generator);
                has_time = all_vehicles[v_index].is_time_feasible_with_recharge(
                                                    request, this->all_stations[pos]);
                // std::cout << "Recharge has_time: " << has_time << std::endl;
                if  (has_time) {
                    // can complete recharge and request in time
                    has_energy = all_vehicles[v_index].is_energy_feasible_with_recharge(
                                                            request, this->all_stations[pos]);
                    // std::cout << "Recharge has_energy: " << has_energy << std::endl;
                    if (has_energy) {
                        // has energy to complete the request after the recharge
                        std::cout << "RECHARGED!\n";
                        all_vehicles[v_index].update_state_of_recharged_vehicle(has_time, has_energy, request);
                        this->all_stations[pos].set_is_used(true);
                        this->all_requests.pop();
                    }
                    else {
                        // does not have energy to complete the request (cant reach station or request)
                        car_pointer = nullptr;
                        acquire_vehicle = true;
                    }
                }
                else {
                    // does not have energy and after recharge it gets late
                    car_pointer = nullptr;
                    acquire_vehicle = true;
                }
            }
        }
        else {
            // vehicle cannot accept the request, New vehicle is needed.
            car_pointer = nullptr;
            acquire_vehicle = true;
        }
    }
}

std::vector<Vehicle> Candidate::get_all_vehicles(){return this->all_vehicles;}
std::queue<Request> Candidate::get_all_requests(){return this->all_requests;}
std::vector<Station> Candidate::get_all_stations(){return this->all_stations;}

double Candidate::get_candidate_cost() {return this->candidate_cost;}
void Candidate::set_candidate_cost(double cost){this->candidate_cost=cost;}

bool Candidate::replace_vehicle(){
    // random initializer
    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());

    // randomize which vehicle will be replaced
    std::uniform_int_distribution<int> rand_vehicle(0, all_vehicles.size()-1);
    int random_vehicle = rand_vehicle(generator);
    Vehicle aux_v = this->all_vehicles[random_vehicle];
    
    // randomize station position for the new vehicle
    std::uniform_int_distribution<int> rand_station(0, this->all_stations.size()-1);
    int random_station = rand_station(generator);
    
    // create a new vehicle
    int new_vehicle_id =aux_v.get_vehicle_id()+100;
    Vehicle new_vehicle(this->all_stations[random_station].get_station_loc(), new_vehicle_id);

    
    std::vector<Request> req_vector = aux_v.get_request_list();
    // mudou a dinamica: agora eu tenho as requests e quero ver se o carro atende
    for (auto request = req_vector.begin(); request != req_vector.end();) {

        // Validating conditions for a given vehicle accepting a request
        bool has_time = new_vehicle.is_time_feasible(*request);
        // std::cout << "has_time: " << has_time << std::endl;
        if  (has_time) {
            // can complete request in time
            bool has_energy = new_vehicle.is_energy_feasible(*request);
            // std::cout << "has_energy: " << has_energy << std::endl;
            if  (has_energy) {
                // has energy to complete the request
                // std::cout   << "\tVehicle: " << car_pointer->get_vehicle_id() << " updated, added " 
                //             << "request: O(" << o.get_x() << ", " << o.get_y() << ") | D(" 
                //             << d.get_x() << ", " << d.get_y() << ") | T = " << t << std::endl;
                new_vehicle.update_state_of_vehicle(has_time, has_energy, *request);
                request++;
            }
            else {
                // does not have energy to complete the request. Recharge needed
                int pos = rand_station(generator);
                has_time = new_vehicle.is_time_feasible_with_recharge(
                                                    *request, this->all_stations[pos]);
                // std::cout << "Recharge has_time: " << has_time << std::endl;
                if  (has_time) {
                    // can complete recharge and request in time
                    has_energy = new_vehicle.is_energy_feasible_with_recharge(
                                                            *request, this->all_stations[pos]);
                    // std::cout << "Recharge has_energy: " << has_energy << std::endl;
                    if (has_energy) {
                        // has energy to complete the request after the recharge
                        std::cout << "RECHARGED!\n";
                        new_vehicle.update_state_of_recharged_vehicle(has_time, has_energy, *request);
                        this->all_stations[pos].set_is_used(true);
                    }
                    else {return false;} // does not have energy to complete the request (cant reach station or request)
                }
                else {return false;} // does not have energy and after recharge it gets late
            }
        }
        else {return false;} // vehicle cannot accept the request, New vehicle is needed.
    }
    this->all_vehicles[random_vehicle] = new_vehicle;
    return true;
}

bool Candidate::steal_request() {
    // if the amount of vehicles is equal to the amount of requests, new vehicles cant be bought
    int num_of_requests=0;
    for (auto vehicle: this->all_vehicles)
        num_of_requests+= vehicle.get_request_list().size();
    if  (num_of_requests == this->all_vehicles.size())
        return false;

    // random initializer
    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());

    // randomize which vehicle will be stolen
    std::uniform_int_distribution<int> rand_vehicle(0, this->all_vehicles.size()-1);
    int random_vehicle = rand_vehicle(generator);
    Vehicle aux_v = this->all_vehicles[random_vehicle];
    std::vector<Request> aux_r = aux_v.get_request_list();

    // randomize which request will be taken
    std::vector<Request> rand_vehicle_requests = this->all_vehicles[random_vehicle].get_request_list();
    int num_of_req_in_vehicle = rand_vehicle_requests.size();
    std::uniform_int_distribution<int> rand_request(0, num_of_req_in_vehicle-1);
    int random_request = rand_request(generator);
    Request aux_req = rand_vehicle_requests[random_request];
    
    // randomize station position for the new vehicle
    std::uniform_int_distribution<int> rand_station(0, this->all_stations.size()-1);
    int random_station = rand_station(generator);
    
    // create a new vehicle
    int new_vehicle_id =aux_v.get_vehicle_id()+200;
    Vehicle new_vehicle(this->all_stations[random_station].get_station_loc(), new_vehicle_id);
    
    bool has_time = new_vehicle.is_time_feasible(aux_req);
    if  (has_time) { // can complete request in time
        bool has_energy = new_vehicle.is_energy_feasible(aux_req);
        if  (has_energy) { // has energy to complete the request
            new_vehicle.update_state_of_vehicle(has_time, has_energy, aux_req);
        }
        else { // does not have energy to complete the request. Recharge needed
            int pos = rand_station(generator);
            has_time = new_vehicle.is_time_feasible_with_recharge(aux_req, this->all_stations[pos]);
            if  (has_time) { // can complete recharge and request in time
                has_energy = new_vehicle.is_energy_feasible_with_recharge(aux_req,this->all_stations[pos]);
                if (has_energy) {// has energy to complete the request after the recharge
                    new_vehicle.update_state_of_recharged_vehicle(has_time, has_energy, aux_req);
                    this->all_stations[pos].set_is_used(true);
                }
                else {return false;} // does not have energy to complete the request (cant reach station or request)
            }
            else {return false;} // does not have energy and after recharge it gets late
        }
    }
    else {return false;} // vehicle cannot accept the request, New vehicle is needed.
    
    if  (num_of_req_in_vehicle == 1) {    // if the stolen car only has one request the car must be replaced
        this->all_vehicles[random_vehicle] = new_vehicle;
        std::cout << "Vehicle " << all_vehicles[random_vehicle].get_vehicle_id() <<" got replaced" << std::endl;
    }
    else {                               // else just put another car in the list
        this->all_vehicles.push_back(new_vehicle);
        std::cout << "Added anotha uan\n";
    }
    
    return true;
}
