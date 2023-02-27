#include "candidate.h"
#include <random>
#include <iostream>

// Candidate1::Candidate1(int num_of_cities, int start){
//     // generate a random solution vector.
//     this->num_of_cities = num_of_cities;
//     this->start = start;
//     this->random_solution = create_random_solution();
//     this->greedy_solution = create_greedy_solution();
// }

// std::vector<int> Candidate1::get_random_solution() {
//     return this->random_solution;
// }

// void Candidate1::set_random_solution(std::vector<int> solution) {
//     this->random_solution = solution;
// }

// std::vector<int> Candidate1::get_greedy_solution() {
//     return this->greedy_solution;
// }

// double Candidate1::get_fit_value() {
//     return this->fit_value;
// }

// void Candidate1::set_fit_value(double value) {
//     this->fit_value = value;
// }

// std::vector<int> Candidate1::create_random_solution() {
//     std::vector<int> sol;
//     std::queue<int> remaining_cities;
//     std::random_device rand_dev;
//     std::mt19937 generator(rand_dev());
//     std::uniform_int_distribution<int> rand_position(1,this->num_of_cities-1);
    
//     // lineup all cities except the starting one
//     sol.push_back(this->start);
//     for (int i=0; remaining_cities.size() < this->num_of_cities-1; i++){
//         sol.push_back(-1);
//         if  (i != this->start)
//             remaining_cities.push(i);
//     }
//     sol[sol.size()-1] = sol[0]; 
    
//     // randomize a valid path
//     int position, current_city;
//     while(!remaining_cities.empty()) {        
//         current_city = remaining_cities.front();
//         remaining_cities.pop();
//         while (true) {
//             position = rand_position(generator);
//             if  (sol[position] == -1){
//                 sol[position] = current_city;
//                 break;
//             }
//         }
//     }
    
//     return sol;
// };

// std::vector<int> Candidate1::create_greedy_solution() {
//     std::vector<int> sol;
//     //TODO: implement greedy solution

//     return sol;
// }


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
    
    int         v_index = 0;            // which vehicle is receiving requests
    int         req_index = 0;          // which request is being used
    bool        acquire_vehicle = true;
    Vehicle*    car_pointer;
    // It is necessary to allocate ALL requests
    //for (Request& request : all_requests){
    
    // for (int i=0; i<this->all_requests.size(); i++) {
    while(!this->all_requests.empty()){
        Request request = this->all_requests.front();
        // verify if a new vehicle is needed
        Point o = request.get_origin();
        Point d = request.get_destination();
        int   t = request.get_pickup_time();
        std::cout << "CLIENT: O(" << o.get_x() << ", " << o.get_y() << ") | D(" 
                      << d.get_x() << ", " << d.get_y() << ") | T = " << t << std::endl;
        if  (acquire_vehicle){
            int pos = rand_station(generator);
            Point station_loc = this->all_stations[pos].get_station_loc();
            std::cout << "Vehicle acquired. V=" << v_index << std::endl;
            car_pointer = new Vehicle(station_loc, v_index++);
            std::cout << pos << "   x=" << station_loc.x << "  y=" << station_loc.y << std::endl;
            this->all_vehicles.push_back(*car_pointer);
            std::cout <<" AQUI FODE TUDO\n";
            acquire_vehicle = false;
        }
        // Validating conditions for a given vehicle accepting a request
        bool has_time = this->all_vehicles[v_index].is_time_feasible(request);
        std::cout << "has_time: " << has_time << std::endl;
        if  (has_time) {
            // can complete request in time
            bool has_energy = this->all_vehicles[v_index].is_energy_feasible(request);
            std::cout << "has_energy: " << has_energy << std::endl;
            if  (has_energy) {
                // has energy to complete the request
                std::cout   << "Vehicle: " << car_pointer->get_vehicle_id() << " updated, added rq=" 
                            << "request: O(" << o.get_x() << ", " << o.get_y() << ") | D(" 
                            << d.get_x() << ", " << d.get_y() << ") | T = " << t << std::endl;
                this->all_vehicles[v_index].update_state_of_vehicle(has_time, has_energy, request);
                std::cout << "QNTS REQEUESTS TEM = " << this->all_vehicles[v_index].get_request_list().size() << std::endl;
                this->all_requests.pop();
            }
            else {
                // does not have energy to complete the request. Recharge needed
                int pos = rand_station(generator);
                has_time = all_vehicles[v_index].is_time_feasible_with_recharge(
                                                    request, this->all_stations[pos]);

                std::cout << "Recharge has_time: " << has_time << std::endl;
                if  (has_time) {
                    
                    // can complete recharge and request in time
                    has_energy = all_vehicles[v_index].is_energy_feasible_with_recharge(
                                                            request, this->all_stations[pos]);
                    std::cout << "Recharge has_energy: " << has_energy << std::endl;
                    if (has_energy) {
                        // has energy to complete the request after the recharge
                        all_vehicles[v_index].update_state_of_recharged_vehicle(has_time, has_energy, request);
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