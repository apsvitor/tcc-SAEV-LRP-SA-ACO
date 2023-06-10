#pragma once
#include "candidate.h"

class SimulatedAnnealingOptimization {
private:
    int         current_iteration;
    double      current_temperature;
    double      best_cost;

    void        __disturb_candidate(
            Candidate *ant,
            std::map <pkey, float> &pheromone_matrix);

    bool        __alter_candidate(Candidate *ant);
    
    double      __probability_of_accepting(double new_cost);


public:
    SimulatedAnnealingOptimization();
    void run(Candidate *ant);
};

/*
#pragma once
#include "constants.h"
#include "candidate.h"
#include <iostream>


class SimulatedAnnealing {
private:
    // parameters and input data
    std::vector<Station>    all_stations;
    std::queue<Request>     all_requests;

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
        std::queue<Request> all_requests
    );

    void print_solution();
    void run();

};
*/