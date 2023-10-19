#include "aco.h"


AntColonyOptimization::AntColonyOptimization(std::vector<Vertex*> vertices_list){
    // input data
    this->vertices_list = vertices_list;
    this->global_best = new Candidate();
    // create the graph-like structure and pheromone matrix from input data
    _create_pheromone_matrix();
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

void AntColonyOptimization::run() {
    // int consecutive_no_improvements = 0;
    for (int iter = 0; iter < aco_c::MAX_ITERATIONS; iter++) {
        Candidate *iteration_best = this->ant_builder();
        std::cout << "==> ITER = " << iter << " | Cost: " << iteration_best->candidate_cost;
        std::cout << std::endl;
        // Local search (Simulated Annealing)
        if  (iteration_best->all_vehicles.size() > 1) {
            SimulatedAnnealingOptimization sa_opt;
            sa_opt.run(iteration_best);
        }
        
        if  (iteration_best->candidate_cost < this->global_best->candidate_cost){
            std::cout << " -> New global best found!";
            delete global_best;
            this->global_best = iteration_best;
            // restart callback counter 
            // consecutive_no_improvements = 0;
        }
        // std::cout << "After SA, global: " << this->global_best->candidate_cost << std::endl;
        // else {
        //     consecutive_no_improvements++;
        // }
        // std::cout << std::endl;
        // if  (consecutive_no_improvements == 80){
        //     std::cout << "No improvements for " << consecutive_no_improvements << " iterations in a row." << std::endl;
        //     break;
        // }
        update_pheromone_trail(iteration_best);
        update_pheromone_trail(this->global_best);
    }
}

Candidate* AntColonyOptimization::ant_builder() {
    Candidate *new_ant, *iteration_best = nullptr;
    double iteration_best_cost = INT64_MAX;
    int best_iteration_is_used[this->s_ind.size()],
        best_iteration_is_done[this->r_ind.size()],
        best_iteration_is_refused[this->r_ind.size()];

    for (int i=0; i<aco_c::MAX_ANTS; i++) {
        new_ant = new Candidate(this->vertices_list, this->s_ind, this->r_ind);
        new_ant->generate_candidate(this->pheromone_matrix);
        if  (new_ant->candidate_cost < iteration_best_cost) {
            if  (iteration_best)
                delete iteration_best;
            iteration_best = new_ant;
            // save iteration best vertice_list state
            for (unsigned int j=0; j<iteration_best->vertices_list.size(); j++){    
                Vertex *v = iteration_best->vertices_list[j];
                if  (v->vertex_type == 'r'){
                    best_iteration_is_done[j - this->s_ind.size()] = static_cast<Request*>(v)->is_done;
                    best_iteration_is_refused[j - this->s_ind.size()] = static_cast<Request*>(v)->is_refused;
                }
                else {
                    best_iteration_is_used[j] = static_cast<Station*>(v)->is_used;
                }
            }
            iteration_best_cost = iteration_best->candidate_cost;
        }
        else {
            delete new_ant;
        }
    }
    std::cout << "ant_builder: ";
    for (unsigned int j=0; j<iteration_best->vertices_list.size(); j++){    
        Vertex *v = iteration_best->vertices_list[j];
        if  (v->vertex_type == 'r'){
            static_cast<Request*>(v)->is_done = best_iteration_is_done[j - this->s_ind.size()] ;
            static_cast<Request*>(v)->is_refused = best_iteration_is_refused[j - this->s_ind.size()];
            std::cout << "[r_" << v->vertex_id << ", " << static_cast<Request*>(v)->is_done << "] | ";
        }
        else {
            static_cast<Station*>(v)->is_used = best_iteration_is_used[j];
            std::cout << "[s_" << v->vertex_id << ", " << static_cast<Station*>(v)->is_used << "] | ";
        }
        
    }
    std::cout << std::endl;
    return iteration_best;
}

void AntColonyOptimization::update_pheromone_trail(Candidate *ant) {
    // pheromone evaporation
    for (auto edge: this->pheromone_matrix)
        this->pheromone_matrix[edge.first] *= (1-aco_c::RO);
    
    std::vector<Vehicle*> all_vehicles = ant->all_vehicles;
    double candidate_quality = 1.0 / (ant->candidate_cost);

    for (auto &vehicle: all_vehicles) {
        std::vector<int> full_path = vehicle->vehicle_path;
        for (unsigned int i=0; i<full_path.size()-1; i++) {
            pci id_i = pci(this->vertices_list[full_path[i]]->vertex_type,
                           this->vertices_list[full_path[i]]->vertex_id);
            pci id_j = pci(this->vertices_list[full_path[i+1]]->vertex_type,
                           this->vertices_list[full_path[i+1]]->vertex_id);
            this->pheromone_matrix[pkey(id_i, id_j)] += candidate_quality;
        }
    }
}


double AntColonyOptimization::get_best_candidate_cost() {
    return this->global_best->candidate_cost;
}