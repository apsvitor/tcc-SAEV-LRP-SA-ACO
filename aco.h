#pragma once
#include "candidate.h"


class AntColonyOptimization {
private:
    // parameters and input data
    std::vector<Station>    all_stations;
    std::queue<Request>     all_requests;
    double                  alpha;          // importance of pheromone
    double                  beta;           // importance of edge
    double                  ro;             // pheromone evaporation factor
    int                     max_ants;
    int                     max_iterations;



   // auxiliary methods
   void                     _ant_builder();
   void                     _update_pheromone_trail();
   void                     _total_cost();


public:
    AntColonyOptimization(
        std::vector<Station>    all_stations,
        std::queue<Request>     all_requests,
        double                  alpha,
        double                  beta,
        double                  ro,
        int                     max_ants,
        int                     max_iterations
    );
    void run();

};