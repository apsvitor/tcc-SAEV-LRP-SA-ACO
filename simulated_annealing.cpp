#include "simulated_annealing.h"

SimulatedAnnealingOptimization::SimulatedAnnealingOptimization() {
    this->current_iteration = 0;
    this->current_temperature = sa_c::INITIAL_T;
}

double SimulatedAnnealingOptimization::__probability_of_accepting(double new_cost) {
    if  (new_cost < this->best_cost)
        return 1.0;
    return exp(-abs(new_cost - this->best_cost) / (this->current_temperature));
}

void SimulatedAnnealingOptimization::run(Candidate *ant) {

    int neighbors_search_space = sa_c::MAX_NEIGHBORS_ITERATIONS;
    while (
        this->current_iteration < sa_c::MAX_ITERATIONS && 
        this->current_temperature > sa_c::MIN_T &&
        neighbors_search_space
    ) {
        // for each temperature level explore the neighbors
        for (int i=0; i<neighbors_search_space; i++) {
            if (__alter_candidate(ant))
                break;
        }

        // stopping conditions update
        this->current_iteration += 1;
        this->current_temperature *= sa_c::COOLING_FACTOR;
        neighbors_search_space *= (1-sa_c::NEIGHBOR_REDUCTION_FACTOR);
    }
}


bool SimulatedAnnealingOptimization::__alter_candidate(Candidate *ant) {
    std::vector<Vehicle*> all_vehicles = ant->get_all_vehicles();
    int num_vehicles = all_vehicles.size();
    if  (num_vehicles == 1)
        return true;
    // randomizes which vehicles are being 2-opted
    int v1_index = random_gen::random_int(0, num_vehicles-1);
    int v2_index = random_gen::random_int(0, num_vehicles-1);
    if  (v1_index == v2_index)
        return true;

    // determines the cutting point as half the path's lenght
    int v1_edge_cut = all_vehicles[v1_index]->vehicle_path.size()/2;
    int v2_edge_cut = all_vehicles[v2_index]->vehicle_path.size()/2;
    
    // stores the original paths
    std::vector<Vertex> v1_original_path, v2_original_path;
    for (auto v: all_vehicles[v1_index]->vehicle_path)
        v1_original_path.push_back(v);
    for (auto v: all_vehicles[v2_index]->vehicle_path)
        v2_original_path.push_back(v);

    // builds the new 2-opted paths
    std::vector<Vertex> v1_new_path, v2_new_path;
    int cont=0;
    int new_size_path_1 = v1_edge_cut + (v2_original_path.size() - v2_edge_cut);
    for (int cont=0, aux=0; cont < new_size_path_1; cont++){
        if  (cont < v1_edge_cut)
            v1_new_path.push_back(v1_original_path[cont]);
        else{
            v1_new_path.push_back(v2_original_path[aux + v2_edge_cut]);
            aux++;
        }
    }
    int new_size_path_2 = v2_edge_cut + (v1_original_path.size() - v1_edge_cut);
    for (int cont=0, aux=0; cont < new_size_path_2; cont++) {
        if  (cont < v2_edge_cut)
            v2_new_path.push_back(v2_original_path[cont]);
        else{
            v2_new_path.push_back(v1_original_path[aux + v1_edge_cut]);
            aux++;
        }
    }

    // validates whether the paths are possible or not
    bool is_valid   = ant->validate_path(v1_new_path) & ant->validate_path(v2_new_path);

    if  (is_valid) {
        // set the paths with the new configuration
        all_vehicles[v1_index]->vehicle_path = v1_new_path;
        all_vehicles[v2_index]->vehicle_path = v2_new_path;

        // calculate the new cost function
        double new_cost = ant->__calculate_candidate_cost();

        // apply the Metropolis method
        double rand_number = random_gen::random_float(0.0, 1.0);
        double metropolis_p = __probability_of_accepting(new_cost);
        
        // accepts the new solution
        if  (metropolis_p > rand_number)
            return true;
    }
    // after this point the new solution is rejected - set the original path back
    all_vehicles[v1_index]->vehicle_path = v1_original_path;
    all_vehicles[v2_index]->vehicle_path = v2_original_path;
    return false;
}
