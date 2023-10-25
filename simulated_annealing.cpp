#include "simulated_annealing.h"

SimulatedAnnealingOptimization::SimulatedAnnealingOptimization() {
    this->current_iteration = 0;
    this->current_temperature = sa_c::INITIAL_T;

    // deep copy ant into cand to preserve ACO's iteration best solution
}

double SimulatedAnnealingOptimization::__probability_of_accepting(double new_cost) {
    if  (new_cost < this->best_cost)
        return 1.0;
    return exp(-abs(new_cost - this->best_cost) / (this->current_temperature));
}

Candidate* SimulatedAnnealingOptimization::run(Candidate *ant) {
    this->cand = new Candidate(*ant);
    int neighbors_search_space = sa_c::MAX_NEIGHBORS_ITERATIONS;
    this->best_cost = this->cand->candidate_cost;
    // useful_prints::printSol(this->cand, "After Copying: ");
    while (this->current_iteration < sa_c::MAX_ITERATIONS && 
           this->current_temperature > sa_c::MIN_T &&
           neighbors_search_space) {

        // for each temperature level explore the neighbors
        for (int i=0; i<neighbors_search_space; i++) {
            bool found_better = alter_candidate();
            if (found_better){
                this->best_cost = this->cand->candidate_cost;
                break;
            }
        }
        // stopping conditions update
        this->current_iteration += 1;
        this->current_temperature *= sa_c::COOLING_FACTOR;
        neighbors_search_space *= (1-sa_c::NEIGHBOR_REDUCTION_FACTOR);
    }
    return this->cand;
}

bool SimulatedAnnealingOptimization::alter_candidate() {
    // randomize two vehicles to have their paths rebuilt
    std::vector<Vehicle*> all_vehicles = this->cand->all_vehicles;
    int num_vehicles = all_vehicles.size(),
        v1_index = random_gen::random_int(0, num_vehicles-1),
        v2_index = random_gen::random_int(0, num_vehicles-1);
    // prevents v2 == v1
    while(v2_index == v1_index){
        v2_index = random_gen::random_int(0, num_vehicles-1);
    }

    // original state (in case the solution gets rejected)
    Vehicle *v1 = all_vehicles[v1_index], *v2 = all_vehicles[v2_index];
    int original_remaining_requests = this->cand->remaining_requests;
    int original_ignored_requests = this->cand->ignored_requests;
    std::vector<int> original_vertices_status;
    std::vector<int> refused_index;
    std::vector<bool> original_is_refused;
    for (unsigned int i=0; i<this->cand->vertices_list.size(); i++) {
        Vertex *v = this->cand->vertices_list[i];
        if  (v->vertex_type == 'r') {
            original_vertices_status.push_back(static_cast<Request*>(v)->is_done);
            original_is_refused.push_back(static_cast<Request*>(v)->is_refused);
            if  (static_cast<Request*>(v)->is_refused)
                refused_index.push_back(i);
        }
        else {
            original_vertices_status.push_back(static_cast<Station*>(v)->is_used);
            original_is_refused.push_back(-1);
        }
    }
    // reset vertices answered by the vehicles
    this->__reset_path(v1, refused_index);
    this->__reset_path(v2, refused_index);

    // chose the requests that will be answered
    this->__chose_new_requests(refused_index);

    // create the new vehicles
    Vehicle *v1_new = this->cand->__generate_new_vehicle(v1->vehicle_id),
            *v2_new = this->cand->__generate_new_vehicle(v2->vehicle_id);

    // create a new path for each vehicle
    this->__build_new_path(v1_new, v2_new);

    // if the new path doesn't answer enough requests, discard solution
    if (this->cand->remaining_requests > 0) {
        for (unsigned int i=0; i<this->cand->vertices_list.size(); i++) {
            Vertex *v = this->cand->vertices_list[i];
            if  (v->vertex_type == 'r') {
                static_cast<Request*>(v)->is_done = original_vertices_status[i];
                static_cast<Request*>(v)->is_refused = original_is_refused[i];
            }
            else {
                static_cast<Station*>(v)->is_used = original_vertices_status[i];
            }
        }
        this->cand->remaining_requests = original_remaining_requests;
        this->cand->ignored_requests = original_ignored_requests;
        return false;
    }

    // evaluate the new solution
    this->cand->all_vehicles[v1_index] = v1_new;
    this->cand->all_vehicles[v2_index] = v2_new;
    double new_cost = this->cand->__calculate_cost();

    // apply the metropolis method
    double rand_number = random_gen::random_float(0.0, 1.0);
    double metropolis_p = __probability_of_accepting(new_cost);

    // accepts the new solution
    if  (metropolis_p > rand_number){
        this->cand->candidate_cost = new_cost;
        return true;
    }
    // reject the new solution
    delete this->cand->all_vehicles[v1_index];
    delete this->cand->all_vehicles[v2_index];
    this->cand->all_vehicles[v1_index] = v1;
    this->cand->all_vehicles[v2_index] = v2;
    for (unsigned int i=0; i<this->cand->vertices_list.size(); i++){
        Vertex* v = this->cand->vertices_list[i];
        if  (v->vertex_type == 'r'){
            static_cast<Request*>(v)->is_done = original_vertices_status[i];
            static_cast<Request*>(v)->is_refused = original_is_refused[i];
        }
        else{
            static_cast<Station*>(v)->is_used = original_vertices_status[i];
        }
    }
    this->cand->remaining_requests = original_remaining_requests;
    this->cand->ignored_requests = original_ignored_requests;
    return false;
}


void SimulatedAnnealingOptimization::__reset_path(Vehicle *car_pointer, std::vector<int>& refused_index) {
    for (unsigned int i=0; i<car_pointer->vehicle_path.size(); i++) {
        Vertex *v = this->cand->vertices_list[car_pointer->vehicle_path[i]];
        if  (v->vertex_type == 'r') {
            static_cast<Request*>(v)->is_done = false;
            static_cast<Request*>(v)->is_refused = false;
            // add requests to the possibilities pool
            refused_index.push_back(car_pointer->vehicle_path[i]);
            this->cand->remaining_requests++;
            // request appear twice on paths
            i++;
        }
        else {
            static_cast<Station*>(v)->is_used -= 1;
        }
    }
}

void SimulatedAnnealingOptimization::__chose_new_requests(const std::vector<int>& refused_index){
    // re-draws the requests that will be answered
    int answer_count = 0, ignore_count = 0,
        max_refused_requests = std::floor(this->cand->r_ind.size() * (1.0 - request_c::MIN_REQUESTS_DONE));
    // indexes relate to the vertice_list vector
    for (auto index: refused_index) {
        Vertex *v = this->cand->vertices_list[index];
        if  (ignore_count == max_refused_requests) {
            // automatically forces the subsequent requests to be answered
            static_cast<Request*>(v)->is_refused = false;
            answer_count++;
        }
        else {
            // randomizes if it will be answered or not
            double rand_accept = random_gen::random_float(0.0, 1.0);
            if  (rand_accept > request_c::ZETA) {
                static_cast<Request*>(v)->is_refused = false;
                answer_count++;
            }
            else {
                static_cast<Request*>(v)->is_refused = true;
                ignore_count++;
            }
        }
    }
    this->cand->remaining_requests = answer_count;
    this->cand->ignored_requests = ignore_count;
}

void SimulatedAnnealingOptimization::__build_new_path(Vehicle *v1, Vehicle *v2) {
    Vehicle *car_pointer = v1;
    for (int iter=0; this->cand->remaining_requests > 0 && iter<2; iter++){
        bool keep_working = true;
        while(keep_working) {
            if  (this->cand->remaining_requests >= 0) {
                Trip trip_chosen;
                Vertex *current_v = this->cand->vertices_list[car_pointer->current_vertex];

                std::vector<int> adj_list_int = current_v->adj_list_int;
                std::vector<Trip> feasible_trips;
                bool request_only_moves_to_stations = true;
                for (auto index: adj_list_int) {
                    Vertex *neighbor_v = this->cand->vertices_list[index];
                    if  (!((neighbor_v->vertex_type == 'r') && (
                            (static_cast<Request*>(neighbor_v)->is_done == true) ||
                            (static_cast<Request*>(neighbor_v)->is_refused == true)))) {
                        Trip trip;
                        try {
                            trip = this->cand->is_feasible(car_pointer, neighbor_v);
                        }
                        catch (const std::exception &error){
                            std::cout << error.what() << std:: endl;
                        }
                        if  (trip.is_feasible){
                            if  (neighbor_v->vertex_type == 'r')
                                request_only_moves_to_stations = false;
                            feasible_trips.push_back(trip);
                        }
                    }
                }
                if  (current_v->vertex_type == 's' && feasible_trips.size() == 0) {
                    trip_chosen = Trip(false, 0, 0, current_v->vertex_id, -1, -1, -1, STATION_STOP);
                }
                else if  (current_v->vertex_type == 'r' && int(feasible_trips.size()) <= this->cand->num_stations && request_only_moves_to_stations) {
                    int random_trip = random_gen::random_int(0, int(feasible_trips.size()-1));
                    trip_chosen = feasible_trips[random_trip];
                }
                else {
                    int random_move = random_gen::random_int(0, int(feasible_trips.size()-1));
                    trip_chosen = feasible_trips[random_move];
                }

                keep_working = trip_chosen.is_feasible;
                if  (keep_working){
                    this->cand->update_vehicle(car_pointer, trip_chosen);
                }
            }
            else {
                keep_working = false;
            }
        }
        car_pointer = v2;
    }
}