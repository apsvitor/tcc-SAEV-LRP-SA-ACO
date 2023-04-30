#include "candidate.h"


Candidate::Candidate(
        std::vector<Vertex*> vertices_list,
        std::vector<int>     s_ind,
        std::vector<int>     r_ind){
    // Indices will serve as reference for RNG
    this->vertices_list = vertices_list;
    this->s_ind         = s_ind;
    this->r_ind         = r_ind;
    this->num_requests  = r_ind.size();

}

int Candidate::__station_randomizer() {
    // returns the index for the station in the vertices list
    int num_stations = this->s_ind.size();
    int s_index =  random_gen::random_int(0, num_stations);
    return this->s_ind[s_index];
}

Vertex* Candidate::__find_a_station_to_stop(Vehicle *car_pointer) {
    Vertex *final_station;
    for (auto ind: this->s_ind) {
        final_station = this->vertices_list[ind];
        bool has_energy = car_pointer->is_energy_feasible(final_station);
        if  (has_energy)
            return final_station;
    }
    // It is assumed that the input will be "friendly enough" to never 
    // reach this part. It must be possible for a vehicle to answer
    // requests and get back to any station given a minimum charge.
    std::cout << "Bad entry on finding a station to stop\n";
    return nullptr;
}

Vehicle* Candidate::__generate_new_vehicle(int v_index) {
    int station_index = __station_randomizer();
    Vertex *starting_point = this->vertices_list[station_index];
    static_cast<Station*>(starting_point)->is_used = true;
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
    
    for (auto neighbor_v: adj_list) {
        pci id_j = pci(neighbor_v->vertex_type, neighbor_v->vertex_id);
        // if it is a request that's already done it musn't be taken into account
        if  (!(neighbor_v->vertex_type == 'r' && static_cast<Request*>(neighbor_v)->is_done == true)) {
            cumulative_sum.push_back(pdi(pheromone_matrix[pkey(id_i, id_j)], index));
            if  (cumulative_sum.size() > 1)
                cumulative_sum[cumulative_sum.size()-1].first += cumulative_sum[cumulative_sum.size()-2].first;
        }
        index++;
    }

    // finds out which edge will be randomly chosen according to its interval
    double max_sum = cumulative_sum[cumulative_sum.size()-1].first;
    double random_probability = random_gen::random_float(0.0, 1.0) * max_sum;
    // TODO: implement a binary search for larger/denser instances
    for (auto ph_ind: cumulative_sum){
        if  (random_probability <= ph_ind.first)
            return current_v->adj_list[ph_ind.second];
    }    
    std::cout << "Bad entry on finding an edge to finish the candidate\n";
    return nullptr;
}

void Candidate::__path_builder(std::map<pkey, float> &pheromone_matrix, 
                               Vehicle *car_pointer) {
    // calls itself repeatedly until a vehicle exhausts its choices.
    Vertex* current_v = car_pointer->current_vertex;
    // make a move
    
    Vertex* next_v = __choose_next_edge(pheromone_matrix, current_v);
    std::cout << "Current: " << current_v->vertex_type << '_' << current_v->vertex_id 
              << " | Chosen: " << next_v->vertex_type << '_' << next_v->vertex_id << '\n';
    // is it possible?
    bool is_on_time = car_pointer->is_time_feasible(next_v);
    bool has_energy = car_pointer->is_energy_feasible(next_v);

    if  (is_on_time && has_energy) {
        std::cout << "Movement succesfully done!\n";
        if  (next_v->vertex_type == 's') {
            static_cast<Station*>(car_pointer->current_vertex)->is_used = true;
            car_pointer->update_vehicle_recharge(next_v);
        }
        else { // is a request
            static_cast<Request*>(next_v)->is_done = true;
            this->num_requests--;
            car_pointer->update_vehicle_request(next_v);
        }
        car_pointer->current_vertex = next_v;
        car_pointer->add_vertex_to_vehicle_path(*next_v);
        if  (this->num_requests)
            __path_builder(pheromone_matrix, car_pointer);
        else {
            std::cout << "No more requests to do. Path-finding to a station.\n";
            Vertex *final_vertex = __find_a_station_to_stop(car_pointer);
            car_pointer->add_vertex_to_vehicle_path(*final_vertex);
            car_pointer->current_vertex = final_vertex;
        }    
    }
    // if the vehicle cant complete the next_v task there are 2 scenarios:
    else if (car_pointer->current_vertex->vertex_type == 's') {
        std::cout << "Already at a station. Ceases all activity.\n";
        // In this case the vehicle is considered to have exhausted its uses.
        // Just return the path that was already built.
        // DUVIDA: devo realmente parar ou abasteço o veículo e retomo a recursão?
        static_cast<Station*>(current_v)->free_spaces += 1;
    }
    else if (car_pointer->current_vertex->vertex_type == 'r') {
        // In this case the vehicle must find a station to port.
        // DUVIDA: devo realmente parar ou faço o backtracking até que a busca por
        //         solução seja finalizada em uma estação (condição acima)?
        Vertex *final_vertex = __find_a_station_to_stop(car_pointer);
        std::cout << "At request r_" << car_pointer->current_vertex->vertex_id << ", tried to go to "
                  << next_v->vertex_type << '_' << next_v->vertex_id << " but couldn't. So it will go to s_"
                  << final_vertex->vertex_id << " instead.\n";
        car_pointer->add_vertex_to_vehicle_path(*final_vertex);
        car_pointer->current_vertex = final_vertex;
    }

    return;
}

void Candidate::generate_candidate(std::map < pkey, float> &pheromone_matrix) {
    int v_index=0;
    Vehicle *car_pointer;
    this->num_requests = this->r_ind.size();
    for (auto v: this->vertices_list) {
        if  (v->vertex_type == 'r')
            static_cast<Request*>(v)->is_done = false;
        else
            static_cast<Station*>(v)->is_used = false;
    }
    while(this->num_requests > 0) {
        std::cout << "Remaining Requests: " << this->num_requests << std::endl;
        // acquire a vehicle
        car_pointer = __generate_new_vehicle(v_index++);
        std::cout << "Vehicle " << car_pointer->vehicle_id << " acquired. Starting at s_" << car_pointer->current_vertex->vertex_id << '\n';
        // build the path for the vehicle
        __path_builder(pheromone_matrix, car_pointer);
        // add vehicle to the solution
        this->all_vehicles.push_back(*car_pointer);
    }

    // calculate the cost function for the candidate
}

std::vector<Vehicle> Candidate::get_all_vehicles(){
    return this->all_vehicles;
}