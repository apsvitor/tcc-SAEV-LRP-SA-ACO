#include "aco.h"

void AntColonyOptimization::_ant_builder(){
    // an ant must contain a full solution
}

void AntColonyOptimization::_update_pheromone_trail(){
    // updates the pheromone matrix

}

void AntColonyOptimization::_total_cost(){
    // calculates the total cost given a candidate solution
}

AntColonyOptimization::AntColonyOptimization(
    std::vector<Station>    all_stations,
    std::queue<Request>     all_requests,
    double                  alpha,
    double                  beta,
    double                  ro,
    int                     max_ants,
    int                     max_iterations){
    // initial parameters
    this->all_stations      = all_stations;
    this->all_requests      = all_requests;
    this->alpha             = alpha;
    this->beta              = beta;
    this->ro                = ro;
    this->max_ants          = max_ants;
    this->max_iterations    = max_iterations;


}
