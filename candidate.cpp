#include "candidate.h"


Candidate::Candidate(
        std::vector<Vertex*> vertices_list,
        std::vector<int>     s_ind,
        std::vector<int>     r_ind){
    // Indices will serve as reference for RNG
    this->vertices_list = vertices_list;
    this->s_ind         = s_ind;
    this->r_ind         = r_ind;

}

int Candidate::__station_randomizer() {
    // returns the index for the station in the vertices list
    int num_stations = this->s_ind.size();
    int s_index =  random_gen::random_int(0, num_stations);
    return this->s_ind[s_index];
}

Vertex* Candidate::__find_a_station_to_stop(Vehicle *&car_pointer) {
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
    return nullptr;
}

Vehicle* Candidate::__generate_new_vehicle(int &v_index) {
    int station_index = __station_randomizer();
    Vertex *starting_point = this->vertices_list[station_index];
    static_cast<Station*>(starting_point)->is_used = true;
    Vehicle* car_pointer = new Vehicle(starting_point, v_index++);
    car_pointer->add_vertex_to_vehicle_path(*starting_point);

    std::cout << "Vehicle acquired: v_id=" << v_index 
              << "\tLeaving Station s_id=" << station_index << '\n';
    
    return car_pointer;
}

Vertex* Candidate::__choose_next_edge(
        std::map<pkey, float> &pheromone_matrix,
        Vertex *current_v) {
    // given a vehicle's current vertex determine all its neighbors
    std::vector<Vertex*> adj_list = current_v->adj_list;

    // cumulative sum to represent the weights of the edges
    int index=-1;    
    std::vector<pdi> cumulative_sum;
    pci id_i = pci(current_v->vertex_type, current_v->vertex_id);
    for (auto neighbor_v: adj_list) {
        index++; 
        pci id_j = pci(neighbor_v->vertex_type, neighbor_v->vertex_id);
        // if it is a request that's already done it shouldn't be taken into account
        if  (neighbor_v->vertex_type == 'r' && static_cast<Request*>(neighbor_v)->is_done)
            continue;
        cumulative_sum.push_back(pdi(pheromone_matrix[pkey(id_i, id_j)], index));
        if  (index > 0)
            cumulative_sum[index].first += cumulative_sum[index-1].first;
    }

    // finds out which edge will be randomly chosen according to its interval
    double max_sum = cumulative_sum[index-1].first;
    double random_probability = random_gen::random_float(0.0, 1.0) * max_sum;
    // TODO: implement a binary search for larger/denser instances
    for (auto ph_ind: cumulative_sum)
        if  (random_probability <= ph_ind.first)
            return current_v->adj_list[ph_ind.second];

    return nullptr;
}

void Candidate::__answer_request(Vertex* request) {
    // remove every reference to the request's vertex once it is done
    int remove_id = request->vertex_id;
    int v_index = 0;
    // remove the request from the vertices_list
    for (auto vertex: this->vertices_list) {
        if  (vertex->vertex_type == 'r' && vertex->vertex_id == remove_id){
            this->vertices_list.erase(this->vertices_list.begin() + v_index);
            // find the position of v_index in the r_ind vector and erase it
            int v_index_finder = 0;
            for (auto ind: this->r_ind) {
                if  (ind == v_index) {
                    this->r_ind.erase(this->r_ind.begin() + v_index_finder);
                    break;
                }
                v_index_finder++;
            }
            break;
        }
        else {
            // remove the edges from all vertices that connect to the request R
            // search through the adj_list of each vertex to find the edge
            int v_index_finder = 0;
            for (auto neighbor: vertex->adj_list) {
                if  (neighbor->vertex_type == 'r' && neighbor->vertex_id == remove_id) {
                    vertex->adj_list.erase(vertex->adj_list.begin() + v_index_finder);
                    break;
                }
                v_index_finder++;
            }
        }
        v_index++;
    }
}

void Candidate::__path_builder(std::map<pkey, float> &pheromone_matrix, Vehicle *&car_pointer) {
    // calls itself repeatedly until a vehicle exhausts its choices.

    Vertex* current_v = car_pointer->current_vertex;
    // make a move
    Vertex* next_v = __choose_next_edge(pheromone_matrix, current_v);

    // is it possible?
    bool is_on_time = car_pointer->is_time_feasible(next_v);
    bool has_energy = car_pointer->is_energy_feasible(next_v);

    if  (is_on_time && has_energy) {
        if  (next_v->vertex_type == 's') {
            static_cast<Station*>(car_pointer->current_vertex)->is_used = true;
            car_pointer->update_vehicle_recharge(next_v);
        }
        else { // is a request
            __answer_request(next_v);
            // TODO: check if is_done is useless.
            static_cast<Request*>(car_pointer->current_vertex)->is_done = true;
            car_pointer->update_vehicle_request(next_v);
        }
        car_pointer->current_vertex = next_v;
        car_pointer->add_vertex_to_vehicle_path(*next_v);
        __path_builder(pheromone_matrix, car_pointer);
        
    }
    // if the vehicle cant complete the next_v task there are 2 scenarios:
    else if (current_v->vertex_type == 's') {
        // In this case the vehicle is considered to have exhausted its uses.
        // Just return the path that was already built.
        // DUVIDA: devo realmente parar ou abasteço o veículo e retomo a recursão?
        static_cast<Station*>(current_v)->free_spaces += 1;
        car_pointer->add_vertex_to_vehicle_path(*current_v);
    }
    else if (current_v->vertex_type == 'r') {
        // In this case the vehicle must find a station to port.
        // DUVIDA: devo realmente parar ou faço o backtracking até que a busca por
        //         solução seja finalizada em uma estação (condição acima)?
        Vertex *final_vertex = __find_a_station_to_stop(car_pointer);
        car_pointer->add_vertex_to_vehicle_path(*final_vertex);
        car_pointer->current_vertex = final_vertex;
    }
    return;
}

void Candidate::generate_candidate(std::map < pkey, float> &pheromone_matrix) {
    int v_index=-1;
    Vehicle *car_pointer;
    while(this->r_ind.size()) {
        // acquire a vehicle
        car_pointer = __generate_new_vehicle(v_index);

        // build the path for the vehicle
        __path_builder(pheromone_matrix, car_pointer);  

        // add vehicle to the solution
        this->all_vehicles.push_back(*car_pointer);
    }

    std::cout << "HABEMUS CANDIDATO????\n";
    for (auto vehicle: this->all_vehicles) {
        std::cout << "Vehicle [" << vehicle.vehicle_id << "]: ";
        for (auto vertex: vehicle.vehicle_path) {
            std::cout << '[' << vertex.vertex_type << '_' << vertex.vertex_id << "] -> ";
        }
        std::cout << '\n';
    }
    // calculate the cost function for the candidate
}


/*
void Candidate::generate_candidate()
{
    // auxiliary part
    int num_of_stations = this->all_stations.size();
    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());
    std::uniform_int_distribution<int> rand_station(0, num_of_stations-1);
    
    int         v_index = -1;               // which vehicle is receiving requests
    int         req_index = 0;              // which request is being used
    bool        acquire_vehicle = true;
    Vehicle*    car_pointer;
    // It is necessary to allocate ALL requests
    while(!this->all_requests.empty()){
        Request request = this->all_requests.front();
        // verify if a new vehicle is needed
        Point o = request.get_origin();
        Point d = request.get_destination();
        int   t = request.get_pickup_time();
        std::cout << "CLIENT LOOKING FOR VEHICLE: O(" << o.x << ", " << o.y << ") | D(" 
                      << d.x << ", " << d.y << ") | T = " << t << std::endl;
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
                            << "request: O(" << o.x << ", " << o.y << ") | D(" 
                            << d.x << ", " << d.y << ") | T = " << t << std::endl;
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
                //             << "request: O(" << o.x << ", " << o.y << ") | D(" 
                //             << d.x << ", " << d.y << ") | T = " << t << std::endl;
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
*/