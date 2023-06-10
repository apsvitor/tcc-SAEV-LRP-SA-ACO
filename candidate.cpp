#include "candidate.h"

Candidate::Candidate() {
    // dummy candidate
    this->candidate_cost = INT32_MAX;
}

Candidate::Candidate(
        std::vector<Vertex*> vertices_list,
        std::vector<int>     s_ind,
        std::vector<int>     r_ind){
    // Indices will serve as reference for RNG
    this->vertices_list = vertices_list;
    this->s_ind         = s_ind;
    this->r_ind         = r_ind;
    this->remaining_requests  = r_ind.size();
    this->ignored_requests = std::floor(this->remaining_requests * (1.0 - request_c::MIN_REQUESTS_DONE));
}

int Candidate::__station_randomizer() {
    // returns the index for the station in the vertices list
    int num_stations = this->s_ind.size();
    int s_index =  random_gen::random_int(0, num_stations-1);
    return this->s_ind[s_index];
}

Vertex* Candidate::__find_a_station_to_stop(Vehicle *car_pointer) {
    Vertex *final_station;
    for (auto ind: this->s_ind) {
        final_station = this->vertices_list[ind];
        double distance = car_pointer->current_vertex->p_xy.get_distance(final_station->p_xy),
               energy   = distance * vehicle_c::CONSUMPTION_RATE;
        if  (car_pointer->current_battery - energy > 0)
            return final_station;
    }
    // It is assumed that the input will be "friendly enough" to never 
    // reach this part. It must be possible for a vehicle to answer
    // requests and get back to any station given a minimum charge.
    std::cout << "Bad entry on finding a station to stop" << std::endl;
    return nullptr;
}

Vehicle* Candidate::__generate_new_vehicle(int v_index) {
    int station_index = __station_randomizer();
    Vertex *starting_point = this->vertices_list[station_index];
    static_cast<Station*>(starting_point)->is_used += 1;
    Vehicle* car_pointer = new Vehicle(starting_point, v_index);
    car_pointer->add_vertex_to_vehicle_path(*starting_point);

    return car_pointer;
}

Vertex* Candidate::__choose_next_edge(
        std::map<pkey, float> &pheromone_matrix,
        Vertex *current_v) {
    // given a vehicle's current vertex determine all its neighbors
    std::vector<Vertex*> adj_list = current_v->adj_list;
    // cumulative sum to represent the weights of the edges
    int index=0;
    std::vector<pdi> cumulative_sum;
    pci id_i = pci(current_v->vertex_type, current_v->vertex_id);

    // auxiliary ACO variables
    double sum_components_value = 0, heuristic_value, pheromone_value;
    for (auto neighbor_v: adj_list) {
        pci id_j = pci(neighbor_v->vertex_type, neighbor_v->vertex_id);
        // if it is a request that's already done it musn't be taken into account
        if  (!(neighbor_v->vertex_type == 'r' && static_cast<Request*>(neighbor_v)->is_done == true)) {
            // heuristic information:
            if  (neighbor_v->vertex_type == 's' && static_cast<Station*>(neighbor_v)->is_used == 0)
                heuristic_value = heuristic_info::OPEN_NEW_STATION;
            else if (neighbor_v->vertex_type == 'r')
                heuristic_value = heuristic_info::ANSWER_REQUEST;
            else
                heuristic_value = heuristic_info::RECHARGE_VEHICLE;
            heuristic_value = pow(heuristic_value, aco_c::BETA);

            // pheromone value
            pheromone_value = pow(pheromone_matrix[pkey(id_i, id_j)], aco_c::ALPHA);

            sum_components_value += (pheromone_value * heuristic_value);
            // fix the value pushed into the cumulative sum to consider heuristic info and pheromone value
            cumulative_sum.push_back(pdi((pheromone_value * heuristic_value), index));
            if  (cumulative_sum.size() > 1)
                cumulative_sum[cumulative_sum.size()-1].first += cumulative_sum[cumulative_sum.size()-2].first;
        }
        index++;
    }
    
    // std::cout << "cum_sum_size: " << cumulative_sum.size() << "  remaining requests: " << this->remaining_requests << std::endl;
    if (cumulative_sum.size() == 0){
        for (auto v: this->vertices_list) {
            if  (v->vertex_type == 'r') {
                std::cout << "[r_" << v->vertex_id << ", " << static_cast<Request*>(v)->is_done << "] | ";
            }
            else{
                std::cout << "[s_" << v->vertex_id << ", " << static_cast<Station*>(v)->is_used << "] | ";
            }
        }std::cout << std::endl;
        throw std::invalid_argument("cumulative vector is empty");
    }

    // finds out which edge will be randomly chosen according to its interval
    double max_sum = (cumulative_sum[cumulative_sum.size()-1].first) / (sum_components_value);
    double random_probability = random_gen::random_float(0.0, 1.0) * max_sum;
    // TODO: implement a binary search for larger/denser instances
    for (auto ph_ind: cumulative_sum){
        if  (random_probability <= (ph_ind.first) / (sum_components_value))
            return current_v->adj_list[ph_ind.second];
    }    
    std::cout << "Bad entry on finding an edge to finish the candidate" << std::endl;
    return nullptr;
}

void Candidate::__path_builder(std::map<pkey, float> &pheromone_matrix, 
                               Vehicle *car_pointer) {
    // calls itself repeatedly until a vehicle exhausts its choices.
    Vertex* current_v = car_pointer->current_vertex;
    // make a move
    Vertex* next_v = __choose_next_edge(pheromone_matrix, current_v);
    // std::cout << "Current: " << current_v->vertex_type << '_' << current_v->vertex_id 
    //           << " | Chosen: " << next_v->vertex_type << '_' << next_v->vertex_id << std::endl;
    // is it possible?
    bool is_on_time = car_pointer->is_time_feasible(next_v);
    bool has_energy = car_pointer->is_energy_feasible(next_v);

    if  (is_on_time && has_energy) {
        // std::cout << "Movement succesfully done!" << std::endl;
        if  (next_v->vertex_type == 's') {
            car_pointer->update_vehicle_request(next_v);
            static_cast<Station*>(next_v)->is_used += 1;
        }
        else { // is a request
            // std::cout << "next_v: r_" << next_v->vertex_id << std::endl;
            // std::cout << "Antes Rrq: " << this->remaining_requests << " | all_vertices: ";
            // for (auto teste: this->vertices_list){
            //     if  (teste->vertex_type == 'r')
            //         std::cout << "[r_" << teste->vertex_id << ", " << static_cast<Request*>(teste)->is_done << "] | ";
            //     else{
            //         std::cout << "[s_" << teste->vertex_id << ", " << static_cast<Station*>(teste)->is_used << "] | ";
            //     }
            // }std::cout << std::endl;
            static_cast<Request*>(next_v)->is_done = true;
            this->remaining_requests--;
            car_pointer->update_vehicle_request(next_v);
            // std::cout << "Depois Rrq: " << this->remaining_requests << " | all_vertices: ";
            // for (auto teste: this->vertices_list){
            //     if  (teste->vertex_type == 'r')
            //         std::cout << "[r_" << teste->vertex_id << ", " << static_cast<Request*>(teste)->is_done << "] | ";
            //     else{
            //         std::cout << "[s_" << teste->vertex_id << ", " << static_cast<Station*>(teste)->is_used << "] | ";
            //     }
            // }std::cout << std::endl;
        }
        car_pointer->current_vertex = next_v;
        car_pointer->add_vertex_to_vehicle_path(*next_v);
        if  (this->remaining_requests > this->ignored_requests){
            __path_builder(pheromone_matrix, car_pointer);
        }
        else if (car_pointer->current_vertex->vertex_type == 'r') {
            // std::cout << "No more requests to do. Path-finding to a station." << std::endl;
            Vertex *final_vertex = __find_a_station_to_stop(car_pointer);
            car_pointer->add_vertex_to_vehicle_path(*final_vertex);
            car_pointer->current_vertex = final_vertex;
            static_cast<Station*>(car_pointer->current_vertex)->is_used += 1;
        }    
    }
    // if the vehicle cant complete the next_v task there are 2 scenarios:
    else if (car_pointer->current_vertex->vertex_type == 's') {
        // std::cout << "Already at a station. Ceases all activity." << std::endl;
        // In this case the vehicle is considered to have exhausted its uses.
        // Just return the path that was already built.
        static_cast<Station*>(current_v)->free_spaces += 1;
    }
    else if (car_pointer->current_vertex->vertex_type == 'r') {
        // In this case the vehicle must find a station to port.
        Vertex *final_vertex = __find_a_station_to_stop(car_pointer);
        // std::cout << "At request r_" << car_pointer->current_vertex->vertex_id << ", tried to go to "
        //           << next_v->vertex_type << '_' << next_v->vertex_id << " but couldn't. So it will go to s_"
        //           << final_vertex->vertex_id << " instead." << std::endl;
        car_pointer->add_vertex_to_vehicle_path(*final_vertex);
        car_pointer->current_vertex = final_vertex;
        static_cast<Station*>(car_pointer->current_vertex)->is_used += 1;
    }

    return;
}

void Candidate::generate_candidate(std::map <pkey, float> &pheromone_matrix) {
    int v_index=0;
    Vehicle *car_pointer;
    this->remaining_requests = this->r_ind.size();
    for (auto &v: this->vertices_list) {
        if  (v->vertex_type == 'r')
            static_cast<Request*>(v)->is_done = false;
        else
            static_cast<Station*>(v)->is_used = 0;
    }
    
    // Concludes candidate generation whenever the minimum threshold is met.
    while(this->remaining_requests != this->ignored_requests) {
        // acquire a vehicle
        car_pointer = __generate_new_vehicle(v_index++);
        // std::cout << "Vehicle " << car_pointer->vehicle_id << " acquired. Starting at s_" << car_pointer->current_vertex->vertex_id << std::endl;
        // build the path for the vehicle
        __path_builder(pheromone_matrix, car_pointer);
        // add vehicle to the solution
        this->all_vehicles.push_back(car_pointer);
    }

    // calculate the cost function for the candidate
    this->candidate_cost = __calculate_candidate_cost();
}

double Candidate::__calculate_candidate_cost() {
    double station_cost     = 0,
           vehicle_cost     = 0,
           trip_cost        = 0,
           penalty_cost     = 0;

    // for each vehicle, adds the cost of acquisition
    vehicle_cost = this->all_vehicles.size() * vehicle_c::COST_PER_VEHICLE;

    // checks which stations were used and adds up their cost
    for (auto station_index: this->s_ind)
        if  (static_cast<Station*>(this->vertices_list[station_index])->is_used)
            station_cost += station_c::COST_PER_STATION;

    // for each vehicle, calculate the trip cost
    for (auto vehicle: this->all_vehicles) {
        // counts the transitions from one vertex to another
        int num_trips = vehicle->vehicle_path.size()-1;
        for (auto vertex: vehicle->vehicle_path) {
            // adds the transition within the request vertex (origin -> destination)
            if  (vertex.vertex_type == 'r')
                num_trips += 1;
        }
        trip_cost += num_trips * request_c::COST_PER_TRIP;
    }
    if  (this->ignored_requests)
        penalty_cost = this->ignored_requests * request_c::UNSERVED_PENALTY;

    return (station_cost + vehicle_cost + trip_cost + penalty_cost);
}

std::vector<Vehicle*> Candidate::get_all_vehicles(){
    return this->all_vehicles;
}

double Candidate::get_candidate_cost() {
    return this->candidate_cost;
}

void Candidate::change_vehicle(int index, Vehicle *new_vehicle, int new_cost){
    this->all_vehicles[index] = new_vehicle;
    this->candidate_cost = new_cost;
}

void Candidate::set_remaining_requests(int new_value){
    this->remaining_requests = new_value;
}

int Candidate::get_remaining_requests(){
    return this->remaining_requests;
}

int Candidate::get_ignored_requests(){
    return this->ignored_requests;
}

bool Candidate::validate_path(std::vector<Vertex> path) {
    // simulates a situation given a full path
    // vehicle starts at path[0], time=0, battery=100%
    int time_of_vehicle = 0;
    double battery      = vehicle_c::MAX_BATTERY;

    bool    is_on_time = true, 
            has_energy = true;
    for (int i=0; i<path.size()-1; i++) {
        // move type: station -> request
        if  (path[i].vertex_type == 's' && path[i+1].vertex_id == 'r') {
            // the vehicle IS RECHARGING
            double distance_to_origin   = path[i].p_xy.get_distance(path[i+1].p_xy);
            double distance_of_request  = static_cast<Request*>(&(path[i+1]))->request_distance;

            int request_starting_time   = static_cast<Request*>(&(path[i+1]))->pickup_time;
            int time_to_finish_request  = std::ceil(distance_of_request/vehicle_c::MEAN_VELOCITY);
            int time_to_reach_origin    = std::ceil(distance_to_origin/vehicle_c::MEAN_VELOCITY);
            int total_time_cost         = time_of_vehicle + time_to_reach_origin;

            double total_energy_cost    = (distance_to_origin + distance_of_request) * vehicle_c::CONSUMPTION_RATE;
            double missing_energy       = vehicle_c::MAX_BATTERY - battery;

            // if the problem allows partial recharge
            if  (problem_type::IS_PARTIAL_RECHARGE) {
                // calculate the minimum energy required to complete the request
                double  min_energy      = vehicle_c::MIN_BATTERY_LEVEL * vehicle_c::MAX_BATTERY,
                        request_energy  = (distance_to_origin + distance_of_request) * vehicle_c::CONSUMPTION_RATE,
                        min_recharge    = min_energy + request_energy - battery;

                // calculate the time necessary to get that minimum charge
                if  (min_recharge > 0) {
                    int min_recharge_time   = std::ceil(min_recharge / vehicle_c::CHARGING_RATE);
                    total_time_cost         += min_recharge_time;

                    // check if its possible to charge more
                    int max_charging_time   = request_starting_time - (time_of_vehicle + time_to_reach_origin);
                    double max_recharge     = std::min(max_charging_time*vehicle_c::CHARGING_RATE, missing_energy);
                    int time_spent_charging = std::ceil(max_recharge / vehicle_c::CHARGING_RATE);

                    total_time_cost += std::max(time_spent_charging, min_recharge_time);
                    battery         += std::max(max_recharge, min_recharge);
                }
            }
            // the problem demands full recharge
            else {
                double missing_battery = vehicle_c::MAX_BATTERY - battery;
                int time_to_fully_charge = missing_battery / vehicle_c::CHARGING_RATE;
                battery = vehicle_c::MAX_BATTERY;

                // time to fully recharge
                total_time_cost = std::max(total_time_cost + time_to_fully_charge, request_starting_time);
            }
            // time feasibility
            if  (total_time_cost > request_starting_time + request_c::LATENESS_EPS)
                return false;

            // energy feasibility is just a check since it passed the time check with min_recharge
            if  (battery - total_energy_cost < vehicle_c::MIN_BATTERY_LEVEL * vehicle_c::MAX_BATTERY)
                return false;
            // update the vehicle simulation
            time_of_vehicle = total_time_cost;
            battery -= total_energy_cost;
        }
        // move type: request -> request
        else if (path[i].vertex_type == 'r' && path[i+1].vertex_type == 'r') {
            int starting_time = static_cast<Request*>(&(path[i+1]))->pickup_time;
            double distance_of_request = static_cast<Request*>(&(path[i+1]))->request_distance;
            double distance_to_origin = path[i].p_xy.get_distance(path[i+1].p_xy);
            int time_to_reach_origin = std::ceil(distance_to_origin / vehicle_c::MEAN_VELOCITY);
            int time_to_finish_request = std::ceil(distance_of_request / vehicle_c::MEAN_VELOCITY);

            // time feasibility
            if  (time_to_reach_origin + time_of_vehicle > starting_time + request_c::LATENESS_EPS)
                return false;
            
            // energy feasibility
            double energy_cost = (distance_to_origin + distance_of_request) * vehicle_c::CONSUMPTION_RATE;
            if  (battery - energy_cost < vehicle_c::MIN_BATTERY_LEVEL * vehicle_c::MAX_BATTERY)
                return false;

            // updates the vehicle simulation
            time_of_vehicle = std::max(starting_time, time_to_reach_origin + time_of_vehicle) + time_to_finish_request;
            battery -= energy_cost;
        }
        // move type: request -> station
        else if (path[i].vertex_type == 'r' && path[i+1].vertex_type == 's') {
            // doesn't require time feasibility check
            double distance_to_station = path[i].p_xy.get_distance(path[i+1].p_xy);
            double energy_cost = distance_to_station * vehicle_c::CONSUMPTION_RATE;
            int time_to_reach_station = std::ceil(distance_to_station / vehicle_c::MEAN_VELOCITY);

            // energy feasibility
            if  (battery - energy_cost < vehicle_c::MIN_BATTERY_LEVEL * vehicle_c::MAX_BATTERY)
                return false;
            
            // updates the vehicle simulation
            time_of_vehicle += time_to_reach_station;
            battery -= energy_cost;
        }
    }
    return (is_on_time & has_energy);
}