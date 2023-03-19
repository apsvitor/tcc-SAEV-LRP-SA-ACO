#include "simulated_annealing.h"
#include <random>

// ----------------------------------------------------------------
// ==================== CLASS CONSTRUCTOR =========================
// ----------------------------------------------------------------
SimulatedAnnealing::SimulatedAnnealing(
    std::vector<Station> all_stations,
    std::queue<Request> all_requests,
    double initial_T, 
    double cooling_factor, 
    double neighbour_reduction_factor, 
    double min_T, 
    int max_iterations, 
    int max_neighbour_iterations) {
    
    // SA hyper parameters
    this->all_stations = all_stations;
    this->all_requests = all_requests;
    this->initial_T = initial_T;
    this->cooling_factor = cooling_factor;
    this->neighbour_reduction_factor = neighbour_reduction_factor;
    this->min_T = min_T;
    this->max_iterations = max_iterations;
    this->max_neighbour_iterations = max_neighbour_iterations;

    // problem parameters
    this->cost_per_vehicle  = 137.0;
    this->cost_per_station  = 2328.76;
    this->cost_per_trip     = 19.37;

    // auxiliary attributes
    this->current_candidate = new Candidate(all_requests, all_stations);
    this->current_candidate->generate_candidate();
    this->current_iteration = 0;
    this->current_cost      = _total_cost(this->current_candidate);
    this->current_T         = initial_T;


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
    total_cost += num_of_vehicles*this->cost_per_vehicle;
    // total trip cost
    for (Vehicle v: vehicles_used) {
        int num_of_trips = v.get_request_list().size();
        total_cost += this->cost_per_trip*num_of_trips;
    }
    // total station cost
    std::vector<Station> stations_used = this->current_candidate->get_all_stations();
    for (Station s: stations_used) {
        if  (s.get_is_used())
            total_cost += this->cost_per_station;
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
    while(
        this->current_iteration < this->max_iterations &&
        this->current_T > this->min_T &&
        this->max_neighbour_iterations > 0) {
        std::cout << "----------> ITERATION " << this->current_iteration << " Current BEST: " << this->best_cost << std::endl;
        // Candidate  best_neighbor_candidate = *(this->best_candidate);
        // double      best_neighbor_cost = this->best_cost;
        // probability of accepting a solution
        std::random_device rand_dev;
        std::mt19937 generator(rand_dev());
        std::uniform_real_distribution<double> f_distr(0,1);

        // For each temperature level, explore the neighbors
        for (int i=0; i<this->max_neighbour_iterations; i++) {
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
        this->current_T *= this->cooling_factor;
        this->max_neighbour_iterations *= (1-this->neighbour_reduction_factor);
    }
    // output the best solution
    print_solution();
}

void SimulatedAnnealing::print_solution() {
    std::vector<Vehicle> vehicles = this->best_candidate->get_all_vehicles();
    std::vector<Station> stations = this->best_candidate->get_all_stations();

    std::cout << "\nStopped Simulation:"
              << "\nCurrent Iteration: " << this->current_iteration
              << "\nCurrent Temperature: " << this->current_T
              << "\nCurrent Neighbors searched: " << this->max_neighbour_iterations << std::endl;

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