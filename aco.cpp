#include "aco.h"


AntColonyOptimization::AntColonyOptimization(std::vector<Vertex*> vertices_list){
    // input data
    this->vertices_list = vertices_list;
    // create the graph-like structure and pheromone matrix from input data
    _create_pheromone_matrix();
}

void AntColonyOptimization::run() {
/**
 * This method executes the ACO algorithm:
 * 1. While current_iteration != max_iterations
 * 2. Create a colony of ants (set of feasible solutions)
 * 3. Call for a local search metaheuristic (Simulated Annealing)
 * 4. Improved ants (solutions) are used to update the pheromone matrix
 * 5. Ouputs the best solution found.
*/
    // STEP 1
    for (int iter = 0; iter < aco_c::MAX_ITERATIONS; iter++) {
        // Building a new generation
        std::vector<Candidate> ant_colony;
        ant_colony = this->_ant_builder(ant_colony);
        
    }

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

std::vector<Candidate> AntColonyOptimization::_ant_builder(std::vector<Candidate> ant_colony) {
    // an ant must contain a full solution
    Candidate *new_ant;
    for (int i=0; i<aco_c::MAX_ANTS; i++) {
        new_ant = new Candidate(this->vertices_list, this->s_ind, this->r_ind);
        // this methods must take into account the pheromones
        // probably I'll have to alter it when it comes to randomization.
        new_ant->generate_candidate(this->pheromone_matrix);
        std::cout << "Candidate[" << i+1 << "]:\n";
        std::vector<Vehicle> all_vehicles = new_ant->get_all_vehicles();
        for (auto vehicle: all_vehicles) {
            std::cout << "Vehicle [" << vehicle.vehicle_id << "]: ";
            for (auto vertex: vehicle.vehicle_path) {
                std::cout << '[' << vertex.vertex_type << '_' << vertex.vertex_id << "] -> ";
            }
            std::cout << '\n';
        }
        std::cout << "----------------------\n";
        ant_colony.push_back(*new_ant);
    }
    return ant_colony;
}

void AntColonyOptimization::_update_pheromone_trail(std::vector<Candidate> ant_colony){
    // updates the pheromone matrix
    for (auto edge: this->pheromone_matrix) {
        // calculate a new pheromone value for a given edge
        // tau(i,j) = (1-ro) + sum( g(s) )
        // g(s) -> evaluation function 
        double sum_g = 0;
        // for (auto ant: ant_colony) {
        //     sum_g += 1/(ant.get_candidate_cost());
        // }
        // this->pheromone_matrix[edge]->second = this->pheromone_matrix[edge].second*(1-this->ro) + sum_g;
    }

}

void AntColonyOptimization::_total_cost(){
    // calculates the total cost given a candidate solution
}