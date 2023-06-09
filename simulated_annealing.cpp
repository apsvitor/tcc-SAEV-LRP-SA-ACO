#include "simulated_annealing.h"

SimulatedAnnealingOptimization::SimulatedAnnealingOptimization() {
    this->current_iteration = 0;
    this->current_temperature = sa_c::INITIAL_T;
}

void SimulatedAnnealingOptimization::__disturb_candidate(
        Candidate *ant,
        std::map <pkey, float> &pheromone_matrix) {
    /*
    disturbance method:
        randomly chooses a vehicle within the candidate
        get all requests which aren't done + the vehicle's request
        rebuilds the vehicle's path
    */
    
    std::vector<Vehicle*> all_vehicles = ant->get_all_vehicles();
    int replace_index = random_gen::random_int(0, int(all_vehicles.size())-1);

    int num_stations = 0;
    std::vector<int> original_station_uses;
    for (auto v: ant->vertices_list){
        if (v->vertex_type == 's'){
            num_stations++;
            original_station_uses.push_back(static_cast<Station*>(v)->is_used);
        }
    }

    {// checkpoint 1    
        std::cout << "CHECK POINT 1\n";
        for (auto v: ant->vertices_list) {
            if  (v->vertex_type == 'r') {
                std::cout << "[r_" << v->vertex_id << ", " << static_cast<Request*>(v)->is_done << "] | ";
            }
            else{
                std::cout << "[s_" << v->vertex_id << ", " << static_cast<Station*>(v)->is_used << "] | ";
            }
        }std::cout << std::endl;
    }
    // find the requests answered by the original vehicle and set them to unanswered
    int requests_done = 0;
    std::vector<int> disturbed_requests_index;
    int disturbed_stations[num_stations] = {0};
    std::cout << "Chosen vehicle path: " << replace_index << " -> ";
    for (auto &vertex: all_vehicles[replace_index]->vehicle_path){
        int vertex_id = vertex.vertex_id;
        std::cout << "[" << vertex.vertex_type << ", " << vertex_id << "] | ";
        if  (vertex.vertex_type == 'r'){
            static_cast<Request*>(ant->vertices_list[vertex_id+num_stations])->is_done = false;
            disturbed_requests_index.push_back(vertex_id+num_stations);
            requests_done++;
        }
        else {
            disturbed_stations[vertex_id] += 1;
            static_cast<Station*>(ant->vertices_list[vertex_id])->is_used -= 1;
        }
    }
    {// checkpoint 2
        std::cout << "\nCHECKPOINT 2" << std::endl;
        for (auto v: ant->vertices_list) {
            if  (v->vertex_type == 'r') {
                std::cout << "[r_" << v->vertex_id << ", " << static_cast<Request*>(v)->is_done << "] | ";
            }
            else{
                std::cout << "[s_" << v->vertex_id << ", " << static_cast<Station*>(v)->is_used << "] | ";
            }
        }std::cout << std::endl;
    }

    int original_rrq = ant->get_remaining_requests();
    ant->set_remaining_requests(original_rrq + requests_done);

    std::cout << "Requests que podem ser recolocadas: " << requests_done + original_rrq << std::endl;

    int new_vehicle_id = all_vehicles[replace_index]->vehicle_id;
    
    Vehicle *new_vehicle = ant->__generate_new_vehicle(new_vehicle_id);
    ant->__path_builder(pheromone_matrix, new_vehicle);

    this->best_cost = ant->get_candidate_cost();
    double new_cost = ant->__calculate_candidate_cost(),
           acceptance_probability = random_gen::random_float(0.0, 1.0),
           calculated_probability = __probability_of_accepting(new_cost);


    {// checkpoint 3
        std::cout << "\nCHECKPOINT 3" << std::endl;
        for (auto v: ant->vertices_list) {
            if  (v->vertex_type == 'r') {
                std::cout << "[r_" << v->vertex_id << ", " << static_cast<Request*>(v)->is_done << "] | ";
            }
            else{
                std::cout << "[s_" << v->vertex_id << ", " << static_cast<Station*>(v)->is_used << "] | ";
            }
        }std::cout << std::endl;
    }

    if  (calculated_probability >= acceptance_probability){
        {// checkpoint 4
            std::cout << "\nCHECKPOINT 4" << std::endl;
            for (auto v: ant->vertices_list) {
                if  (v->vertex_type == 'r') {
                    std::cout << "[r_" << v->vertex_id << ", " << static_cast<Request*>(v)->is_done << "] | ";
                }
                else{
                    std::cout << "[s_" << v->vertex_id << ", " << static_cast<Station*>(v)->is_used << "] | ";
                }
            }std::cout << std::endl;
        }
        // std::cout << "-> -> -> Vehicle Replaced id: "<< new_vehicle_id << " <- <- <-" << std::endl;
        ant->change_vehicle(replace_index, new_vehicle, new_cost);
        this->best_cost = new_cost;
    }
    else {
        {// checkpoint 5 
            std::cout << "\nCHECKPOINT 5" << std::endl;
            for (auto v: ant->vertices_list) {
                if  (v->vertex_type == 'r') {
                    std::cout << "[r_" << v->vertex_id << ", " << static_cast<Request*>(v)->is_done << "] | ";
                }
                else{
                    std::cout << "[s_" << v->vertex_id << ", " << static_cast<Station*>(v)->is_used << "] | ";
                }
            }std::cout << std::endl;
        }
        // just set the vertices to their original state
        ant->set_remaining_requests(original_rrq);
        for (auto index: disturbed_requests_index){
            static_cast<Request*>(ant->vertices_list[index])->is_done = true;
        }
        int pos = 0;
        for (auto value: disturbed_stations){
            static_cast<Station*>(ant->vertices_list[pos])->is_used = original_station_uses[pos];
            pos++;
        }
        {// checkpoint 6
            std::cout << "\nCHECKPOINT 6" << std::endl;
            for (auto v: ant->vertices_list) {
                if  (v->vertex_type == 'r') {
                    std::cout << "[r_" << v->vertex_id << ", " << static_cast<Request*>(v)->is_done << "] | ";
                }
                else{
                    std::cout << "[s_" << v->vertex_id << ", " << static_cast<Station*>(v)->is_used << "] | ";
                }
            }std::cout << std::endl;
        }
    }
}

double SimulatedAnnealingOptimization::__probability_of_accepting(double new_cost) {
    if  (new_cost < this->best_cost)
        return 1.0;
    return exp(-abs(new_cost - this->best_cost) / (this->current_temperature));
}

void SimulatedAnnealingOptimization::run(
    Candidate *ant,
    std::map <pkey, float> &pheromone_matrix) {
    int neighbors_search_space = sa_c::MAX_NEIGHBORS_ITERATIONS;
    while (
        this->current_iteration < sa_c::MAX_ITERATIONS && 
        this->current_temperature > sa_c::MIN_T &&
        neighbors_search_space
    ) {
        // for each temperature level explore the neighbors
        for (int i=0; i<neighbors_search_space; i++) {
            // disturb a candidate
            // for (auto v: ant->vertices_list) {
            //     if  (v->vertex_type == 'r') {
            //         std::cout << "[r_" << v->vertex_id << ", " << static_cast<Request*>(v)->is_done << "] | ";
            //     }
            //     else{
            //         std::cout << "[s_" << v->vertex_id << ", " << static_cast<Station*>(v)->is_used << "] | ";
            //     }
            // }std::cout << std::endl;
            std::cout << "====================> DISTURBANCE CALLED\n";
            __disturb_candidate(ant, pheromone_matrix);
        }

        // stopping conditions update
        this->current_iteration += 1;
        this->current_temperature *= sa_c::COOLING_FACTOR;
        neighbors_search_space *= (1-sa_c::NEIGHBOR_REDUCTION_FACTOR);
    }
}

/*
#include "simulated_annealing.h"
#include <random>

// ----------------------------------------------------------------
// ==================== CLASS CONSTRUCTOR =========================
// ----------------------------------------------------------------
SimulatedAnnealing::SimulatedAnnealing(
    std::vector<Station> all_stations,
    std::queue<Request> all_requests
    ) {
    
    this->all_stations = all_stations;
    this->all_requests = all_requests;

    // auxiliary attributes
    TODO: SA não gerará um candidato inicial. O ACO se encarregará disso.
    Provavelmente o current_candidate receberá um parâmetro no construtor.
    this->current_candidate = new Candidate(all_requests, all_stations);
    this->current_candidate->generate_candidate();
    //===========OFF
    //this->current_candidate = new_ant;
    this->current_iteration = 0;
    this->current_cost      = _total_cost(this->current_candidate);
    this->current_T         = sa_c::INITIAL_T;


    this->best_candidate    = current_candidate;
    this->best_cost         = this->current_cost;

}
// ----------------------------------------------------------------
// ================ AUXILIARY PRIVATE METHODS =====================
// ----------------------------------------------------------------
double SimulatedAnnealing::_total_cost(Candidate* cand) {
    double total_cost = 0.0;
    std::vector<Vehicle> vehicles_used = cand->get_all_vehicles();
    // total vehicle cost
    int num_of_vehicles = vehicles_used.size();
    total_cost += vehicle_c::COST_PER_VEHICLE * num_of_vehicles;
    // total trip cost
    for (Vehicle v: vehicles_used) {
        int num_of_trips = v.get_request_list().size();
        total_cost += request_c::COST_PER_TRIP * num_of_trips;
    }
    // total station cost
    std::vector<Station> stations_used = this->current_candidate->get_all_stations();
    for (Station s: stations_used) {
        if  (s.get_is_used())
            total_cost += station_c::COST_PER_STATION;
    }
    // saving total cost for the candidate
    cand->set_candidate_cost(total_cost);
    return total_cost;
}

double SimulatedAnnealing::_probability_of_accepting(double new_cost) {
    if  (new_cost < this->best_cost)
        return 1.0;
    return exp(-abs(new_cost-this->best_cost) / (this->current_T));
}

Candidate SimulatedAnnealing::_disturb_candidate(Candidate* cand) {
    // generate a random number  between 0 and 1
    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());
    std::uniform_real_distribution<double> disturb(0,1);
    double disturbance_rate = disturb(generator);

    if  (disturbance_rate < 0.9) {
        // create a new vehicle, with new random starting station and take the requests
        // from a random existing vehicle (and replace it)
        std::cout << "Replace called\n";
        bool is_replaced = cand->replace_vehicle();
        // if (is_replaced)
        //     std::cout << "Successfully replaced vehicle\n";
        // else
        //     std::cout << "Replace attempt failed. Infeasible solution\n";
    }
    // else {

    // TODO: FIX THIS METHOD
    //     // criar novo veiculo e roubar uma request de um aleatorio
    //     std::cout << "Steal called\n";
    //     bool is_stolen = cand->steal_request();
    //     // if (is_stolen)
    //     //     std::cout << "Successfully created new vehicle and stolen request\n";
    //     // else
    //     //     std::cout << "Infeasible solution. Too many vehicles\n";
    // }
    double new_cost = _total_cost(cand);
    return *cand;
}

// ----------------------------------------------------------------
//
// ----------------------------------------------------------------

void SimulatedAnnealing::run() {
    // Run until one or more conditions are no longer true
    int neighbors_search_space = sa_c::MAX_NEIGHBORS_ITERATIONS;
    while(
        this->current_iteration < sa_c::MAX_ITERATIONS &&
        this->current_T > sa_c::MIN_T &&
        neighbors_search_space > 0) {
        std::cout << "----------> ITERATION " << this->current_iteration << " Current BEST: " << this->best_cost << std::endl;
        // Candidate  best_neighbor_candidate = *(this->best_candidate);
        // double      best_neighbor_cost = this->best_cost;
        // probability of accepting a solution
        std::random_device rand_dev;
        std::mt19937 generator(rand_dev());
        std::uniform_real_distribution<double> f_distr(0,1);

        // For each temperature level, explore the neighbors
        for (int i=0; i<neighbors_search_space; i++) {
            // find a new neighbor
            Candidate* neighbor_sol = new Candidate(this->_disturb_candidate(this->current_candidate));
            double neighbor_cost = neighbor_sol->get_candidate_cost();
            
            double p_to_accept = f_distr(generator);
            double p_calculated = this->_probability_of_accepting(neighbor_cost);
            // if the solution is accepted, store it as the best neighbor solution yet
            if  (p_calculated >= p_to_accept) {
                this->current_candidate = neighbor_sol;
                this->current_cost = neighbor_cost;
            }
        // change the best for the best neighbor
        this->best_candidate = this->current_candidate;
        this->best_cost = this->current_cost;
        
        }

        // alter the state
        this->current_iteration += 1;
        this->current_T *= sa_c::COOLING_FACTOR;
        neighbors_search_space *= (1-sa_c::NEIGHBOR_REDUCTION_FACTOR);
    }
    // output the best solution
    print_solution();
}

void SimulatedAnnealing::print_solution() {
    std::vector<Vehicle> vehicles = this->best_candidate->get_all_vehicles();
    std::vector<Station> stations = this->best_candidate->get_all_stations();

    std::cout << "\nStopped Simulation:"
              << "\nCurrent Iteration: " << this->current_iteration
              << "\nCurrent Temperature: " << this->current_T << '\n';
            //   << "\nCurrent Neighbors searched: " << this->max_neighbour_iterations << std::endl;

    std::cout << "Best Solution Found:\n"
              << "Cost: " << this->best_cost << std::endl
              << "Stations used:\n";
    for (Station station : stations) {
        if  (station.get_is_used()){
            Point location = station.get_station_loc();
            std::cout << "\tStation: " << station.get_station_id()
                      << " (" << location.x << ", " << location.y << ")\n";
        }
    }
    std::cout << "Vehicles and Request Distribution:\n";
    for (Vehicle vehicle : vehicles) {
        std::vector<Request> requests = vehicle.get_request_list();
        std::cout << "Vehicle " << vehicle.get_vehicle_id() << ":\n";
        for (Request req : requests) {
            Point origin = req.get_origin();
            Point destination = req.get_destination();
            double pickup_time = req.get_pickup_time();
            int request_id = req.get_request_id();
            std::cout << "\tRequest " << request_id << ": | " 
                      << "O(" << origin.x << ", " << origin.y << ") | "
                      << "D(" << destination.x << ", " << destination.y << ") | "
                      << "Pickup time: " << pickup_time << std::endl;
        }
    }
}

*/