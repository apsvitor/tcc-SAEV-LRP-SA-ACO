#pragma once

#include "candidate.h"
#include <iostream>


class SimulatedAnnealing {
private:
    // parameters and input data
    std::vector<Station>    all_stations;
    std::queue<Request>     all_requests;
    double                  initial_T; 
    double                  cooling_factor; 
    double                  neighbour_reduction_factor; 
    double                  min_T;
    int                     max_iterations; 
    int                     max_neighbour_iterations;

    // problem parameters
    double                  cost_per_vehicle;
    double                  cost_per_station;
    double                  cost_per_trip;

    // auxiliary attributes
    int                     current_iteration;
    double                  current_T;
    double                  current_cost;
    Candidate*              current_candidate;

    // the answer
    Candidate*              best_candidate;
    double                  best_cost;


    // private cost method
    double                  _total_cost(Candidate* cand);
    double                  _probability_of_accepting(double new_cost);
    Candidate               _disturb_candidate(Candidate* cand);


public:
    SimulatedAnnealing(
        std::vector<Station> all_stations,
        std::queue<Request> all_requests,
        double initial_T = 100.0, 
        double cooling_factor = 0.99, 
        double neighbour_reduction_factor = 0.05, 
        double min_T = 1.0, 
        int max_iterations = 100, 
        int max_neighbour_iterations = 50
    );

    void print_solution();
    void run();


};