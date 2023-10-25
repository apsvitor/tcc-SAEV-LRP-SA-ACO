#include "aco.h"


AntColonyOptimization::AntColonyOptimization(std::vector<Vertex*> vertices_list){
    // input data
    this->vertices_list = vertices_list;
    this->global_best = nullptr; // Candidate();
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
        int neighbor_index = 0;
        for (auto &neighbor: this->vertices_list) {
            neighbor_id = neighbor->vertex_id;
            neighbor_type = neighbor->vertex_type;
            is_neighbor = (neighbor_type == 'r');
            if  (vertex_type == 'r')
                is_neighbor = !(is_neighbor && neighbor_id == vertex_id);
            
            if  (is_neighbor) {
                id_j = pci(neighbor_type, neighbor_id);
                this->pheromone_matrix[pkey(id_i, id_j)] = aco_c::BASE_PHEROMONE;
                // vertex->adj_list.push_back(neighbor);
                vertex->adj_list_int.push_back(neighbor_index);
            }
            neighbor_index++;
        }
    }
}

void AntColonyOptimization::run() {
    // int consecutive_no_improvements = 0;
    for (int iter = 0; iter < aco_c::MAX_ITERATIONS; iter++) {
        
        Candidate *iteration_best = this->ant_builder();
        Candidate *sa_best;
        std::cout << "==> ITER = " << iter << " | Cost: " << iteration_best->candidate_cost;
        // Local search (Simulated Annealing)
        if  (iteration_best->all_vehicles.size() > 1) {
            SimulatedAnnealingOptimization sa_opt;
            sa_best = sa_opt.run(iteration_best);
            delete iteration_best;
            iteration_best = sa_best;
        }
        std::cout << " | After SA Cost: " << iteration_best->candidate_cost;
        update_pheromone_trail(iteration_best);

        if  (this->global_best == nullptr || iteration_best->candidate_cost < this->global_best->candidate_cost){
            std::cout << " -> New global best found!";
            if  (this->global_best)
                delete global_best;
            this->global_best = iteration_best;
            // restart callback counter 
            // consecutive_no_improvements = 0;
        }
        else {
            // consecutive_no_improvements++;
            delete iteration_best;
        }
        std::cout << std::endl;
        // if  (consecutive_no_improvements == 80){
        //     std::cout << "No improvements for " << consecutive_no_improvements << " iterations in a row." << std::endl;
        //     break;
        // }
        update_pheromone_trail(this->global_best);
    }
}

Candidate* AntColonyOptimization::ant_builder() {
    Candidate *new_ant, *iteration_best = nullptr;
    double iteration_best_cost = INT64_MAX;
    std::vector<int> _vertice_status;
    std::vector<bool> _is_refused;
    // int best_iteration_is_used[this->s_ind.size()],
        // best_iteration_is_done[this->r_ind.size()],
        // best_iteration_is_refused[this->r_ind.size()];

    for (int i=0; i<aco_c::MAX_ANTS; i++) {
        new_ant = new Candidate(this->vertices_list, this->s_ind, this->r_ind);
        new_ant->generate_candidate(this->pheromone_matrix);

        if  (new_ant->candidate_cost < iteration_best_cost) {
            if  (iteration_best)
                delete iteration_best;
            iteration_best = new_ant;
            _vertice_status.clear();
            _is_refused.clear();
            // save iteration best vertice_list state
            for (unsigned int j=0; j<iteration_best->vertices_list.size(); j++){    
                Vertex *v = iteration_best->vertices_list[j];
                if  (v->vertex_type == 'r'){
                    _vertice_status.push_back(static_cast<Request*>(v)->is_done);
                    _is_refused.push_back(static_cast<Request*>(v)->is_refused);
                    // best_iteration_is_done[j - this->s_ind.size()] = static_cast<Request*>(v)->is_done;
                    // best_iteration_is_refused[j - this->s_ind.size()] = static_cast<Request*>(v)->is_refused;
                }
                else {
                    _vertice_status.push_back(static_cast<Station*>(v)->is_used);
                    _is_refused.push_back(-1);
                    // best_iteration_is_used[j] = static_cast<Station*>(v)->is_used;
                }
            }
            iteration_best_cost = iteration_best->candidate_cost;
        }
        else {
            delete new_ant;
        }
    }
    for (unsigned int j=0; j<iteration_best->vertices_list.size(); j++){    
        Vertex *v = iteration_best->vertices_list[j];
        if  (v->vertex_type == 'r'){
            static_cast<Request*>(v)->is_done = _vertice_status[j];
            static_cast<Request*>(v)->is_refused = _is_refused[j];
        }
        else {
            static_cast<Station*>(v)->is_used = _vertice_status[j];
        }
        
    }
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