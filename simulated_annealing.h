#pragma once

#include "candidate.h"
#include <iostream>

class SimulatedAnnealing {
private:
    std::vector<Point>                  coordinates;
    std::vector< std::vector<double> >  distance_matrix;
    int                                 num_of_cities;
    int                                 starting_city;

    double      initial_T;
    double      cooling_factor;
    double      neighbour_reduction_factor;
    double      min_T;
    int         max_iterations;
    int         max_neighbour_iterations;

    double      current_T;
    int         current_iteration;
    int         current_neighbors_searched;

    double      best_cost;
    Candidate   best_candidate;


    double      _fitness(Candidate* candidate);
    double      _probability_of_accepting(Candidate* new_solution);
    

public:
    // SimulatedAnnealing() {};
    SimulatedAnnealing(
        std::vector<Point> coordinates, 
        std::vector< std::vector<double> > distance_matrix, 
        int starting_city, 
        double initial_T = 100.0, 
        double cooling_factor = 0.99, 
        double neighbour_reduction_factor = 0.05, 
        double min_T = 1.0, 
        int max_iterations = 100, 
        int max_neighbour_iterations = 50);
    void run();


};
