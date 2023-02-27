#include "simulated_annealing.h"
#include <cmath>

// SimulatedAnnealing::SimulatedAnnealing(
//     std::vector<Point> coordinates,
//     std::vector< std::vector<double> > distance_matrix, 
//     int starting_city, 
//     double initial_T, 
//     double cooling_factor, 
//     double neighbour_reduction_factor, 
//     double min_T, 
//     int max_iterations, 
//     int max_neighbour_iterations
// ) {
//     this->coordinates = coordinates;
//     this->distance_matrix = distance_matrix;
//     this->num_of_cities = coordinates.size();
//     this->starting_city = starting_city;

//     this->initial_T = initial_T;
//     this->cooling_factor = cooling_factor;
//     this->neighbour_reduction_factor = neighbour_reduction_factor;
//     this->min_T = min_T;
//     this->max_iterations = max_iterations;
//     this->max_neighbour_iterations = max_neighbour_iterations;

//     this->current_T = initial_T;
//     this->current_iteration = 0;
//     this->current_neighbors_searched = 10*this->num_of_cities;

//     this->best_candidate = Candidate1(this->num_of_cities, this->starting_city);
//     this->best_cost = this->_fitness(&best_candidate);
// }


// double SimulatedAnnealing::_fitness(Candidate1* candidate) {
//     double fit_value = 0.0;
//     std::vector<int> sol = candidate->get_random_solution();
//     for (int i = 0; i < sol.size()-1; i++)
//         fit_value += this->distance_matrix[sol[i]][sol[i+1]];
//     candidate->set_fit_value(fit_value);
//     return fit_value;
// }

// double SimulatedAnnealing::_probability_of_accepting(Candidate1* new_solution) {
//     double new_cost = new_solution->get_fit_value();
//     std::cout << "NEW_COST=" << new_cost << "| BEST_COST = " << this->best_cost << std::endl;
//     if  (new_cost < this->best_cost)
//         return 1.0;
    
//     return exp(-abs(new_cost-this->best_cost) / (this->current_T));
// }


// void SimulatedAnnealing::run() {

//     // Run until one of the conditions are no longer true
//     while (
//         this->current_iteration < this->max_iterations && 
//         this->current_T > this->min_T &&
//         this->current_neighbors_searched > 0) {
//         // Current iteration
//         std::cout << "Current iteration: " << this->current_iteration 
//                   << " | Best Cost: " << this->best_cost << std::endl
//                   << "\tCurrent best_path: ";
//         for (auto city: this->best_candidate.get_random_solution())
//             std::cout << city << " -> ";
//         std::cout << std::endl;
        
//         std::vector<int> current_path = this->best_candidate.get_random_solution();

//         // Temporary neighbor solution stored until all neighbors are searched 
//         Candidate1 best_neighbor_candidate = this->best_candidate;
//         std::vector<int> best_neighbor_path = this->best_candidate.get_random_solution();
//         double best_neighbor_cost = this->best_cost;

//         // Generate a new (neighbor) solution for the current temperature
//         for (int i = 0; i < this->current_neighbors_searched; i++) {
            
//             // random integer number generator
//             std::random_device rand_dev;
//             std::mt19937 generator(rand_dev());
//             std::uniform_int_distribution<int> distr(1, current_path.size()-2);

//             // 2-opt
//             int pos_1 = distr(generator);
//             int pos_2 = distr(generator);
//             int temp = current_path[pos_1];
            
//             // neighbor candidate solution
//             Candidate1 new_candidate = this->best_candidate;
//             std::vector<int> candidate_path = new_candidate.get_random_solution();
//             candidate_path[pos_1] = candidate_path[pos_2];
//             candidate_path[pos_2] = temp;
//             new_candidate.set_random_solution(candidate_path);
            
//             double new_candidate_cost = this->_fitness(&new_candidate);

//             // probability of accepting a solution
//             std::uniform_real_distribution<double> f_distr(0,1);
//             double p_to_accept = f_distr(generator);
//             double p_calculated = this->_probability_of_accepting(&new_candidate);

//             std::cout << "\t\tTrocou -> p_calculado =  " << p_calculated << " | p_to_accept = " << p_to_accept << '\n';

//             // if the solution is accepted, store it as the best neighbor solution yet
//             if  (p_calculated == 1.0 || p_calculated < p_to_accept) {
//                 best_neighbor_candidate = new_candidate;
//                 best_neighbor_cost = new_candidate_cost;
//             }
//         }

//         // Replace the current best with the neighbor chosen previously
//         this->best_candidate = best_neighbor_candidate;
//         this->best_cost = best_neighbor_cost;

//         this->current_iteration += 1;
//         this->current_T *= this->cooling_factor;
//         this->current_neighbors_searched *= (1-this->neighbour_reduction_factor);
//     }

//     std::cout << "Stopped at iteration " << this->current_iteration << std::endl
//               << "Current temperature: " << this->current_T << std::endl
//               << "Neighbors searched for that temperature level: " << this->current_neighbors_searched << std::endl
//               << "Current best cost: " << this->best_cost << std::endl
//               << "Solution path found: ";
//     for (auto city: this->best_candidate.get_random_solution()) {
//         std::cout << city << " -> ";
//     }
//     std::cout << std::endl;
// }