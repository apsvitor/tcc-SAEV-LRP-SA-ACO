#include "aco.h"


AntColonyOptimization::AntColonyOptimization(std::vector<Vertex*> vertices_list){
    // input data
    this->vertices_list = vertices_list;
    this->global_best = new Candidate();
    this->iteration_best = global_best;
    // create the graph-like structure and pheromone matrix from input data
    _create_pheromone_matrix();
}

void AntColonyOptimization::run() {
    for (int iter = 0; iter < aco_c::MAX_ITERATIONS; iter++) {
        std::cout << "=========== ITER = " << iter << "===========" << std::endl;
        std::vector<Candidate*> ant_colony;
        ant_colony = this->_ant_builder(ant_colony);
        // Local search (Simulated Annealing)
        SimulatedAnnealingOptimization sa_opt;
        // std::cout << "Cost before SA: " << this->iteration_best->get_candidate_cost() << std::endl;
        sa_opt.run(this->iteration_best);
        // std::cout << "Cost after SA: " << this->iteration_best->get_candidate_cost() << std::endl;
        // for (auto v: this->vertices_list){
        //     if  (v->vertex_type == 'r' && static_cast<Request*>(v)->is_done){
        //         std::cout << v->vertex_type << '_' << v->vertex_id << " | ";
        //     }
        // }std::cout << std::endl;
        // Update the pheromone matrix
        _update_pheromone_trail();
        
    }
    std::cout << "BEST GLOBAL FOUND: RMB$ " << this->global_best->get_candidate_cost() << std::endl;
}

void AntColonyOptimization::_create_pheromone_matrix() {
    // Creates valid edges between the vertices and setup the pheromone matrix
    int ind = 0;
    for (auto &vertex: this->vertices_list) {
        int  neighbor_id,   vertex_id = vertex->vertex_id;
        char neighbor_type, vertex_type = vertex->vertex_type;
        pci  id_i = pci(vertex_type, vertex_id);
        pci  id_j;
        bool is_neighbor;
        // saves the indices for stations and requests within the vertices list
        vertex_type == 'r' ? this->r_ind.push_back(ind) : this->s_ind.push_back(ind);
        ind++;
        for (auto &neighbor: this->vertices_list) {
            neighbor_id = neighbor->vertex_id;
            neighbor_type = neighbor->vertex_type;
            is_neighbor = (neighbor_type == 'r');
            if  (vertex_type == 'r')
                is_neighbor = !(is_neighbor && neighbor_id == vertex_id);
            
            if  (is_neighbor) {
                id_j = pci(neighbor_type, neighbor_id);
                this->pheromone_matrix[pkey(id_i, id_j)] = aco_c::BASE_PHEROMONE;
                vertex->adj_list.push_back(neighbor);
            }
        }
    }

}

std::vector<Candidate*> AntColonyOptimization::_ant_builder(std::vector<Candidate*> ant_colony) {
    // an ant must contain a full solution
    Candidate *new_ant;
    double iteration_best_cost = INT64_MAX;
    for (int i=0; i<aco_c::MAX_ANTS; i++) {
        new_ant = new Candidate(this->vertices_list, this->s_ind, this->r_ind);
        new_ant->generate_candidate(this->pheromone_matrix);
        std::cout << "Candidate[" << i+1 << "] cost: RMB$ " << new_ant->get_candidate_cost() << std::endl;
        std::vector<Vehicle*> all_vehicles = new_ant->get_all_vehicles();
        
        // { // print results
        // for (auto vehicle: all_vehicles) {
        //     std::cout << "Vehicle [" << vehicle->vehicle_id << "]: ";
        //     for (auto vertex: vehicle->vehicle_path) {
        //         std::cout << '[' << vertex.vertex_type << '_' << vertex.vertex_id << "] -> ";
        //     }
        //     std::cout << std::endl;
        // }std::cout << "----------------------" << std::endl;}


        ant_colony.push_back(new_ant);
        double current_ant_cost = new_ant->get_candidate_cost();
        if  (current_ant_cost < iteration_best_cost) {
            iteration_best_cost = current_ant_cost;
            this->iteration_best = new_ant;
        }
        if  (current_ant_cost < this->global_best->get_candidate_cost()){
            std::cout << "New Global Best Found: " << std::endl;
            this->global_best = new_ant;
        }
    }
    return ant_colony;
}

void AntColonyOptimization::_update_pheromone_trail(){
    // updates the pheromone matrix
    // pheromone evaporation
    for (auto edge: this->pheromone_matrix)
        this->pheromone_matrix[edge.first] *= (1-aco_c::RO);

    // deposit pheromone on the best paths
    Candidate* ant = this->global_best;
    for (int i=0; i<2; i++) {
        std::vector<Vehicle*> all_vehicles = ant->get_all_vehicles();
        double candidate_quality = 1 / (ant->get_candidate_cost());

        for (auto &vehicle: all_vehicles) {
            std::vector<Vertex> full_path = vehicle->vehicle_path;

            for (int i=0; i<full_path.size()-1; i++) {
                pci id_i = pci(full_path[i].vertex_type, full_path[i].vertex_id);
                pci id_j = pci(full_path[i+1].vertex_type, full_path[i+1].vertex_id);
                this->pheromone_matrix[pkey(id_i, id_j)] += candidate_quality;
            }
        }
        ant = this->iteration_best;
    }
}
