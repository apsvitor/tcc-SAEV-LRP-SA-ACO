#pragma once
#include "candidate.h"

class SimulatedAnnealingOptimization {
private:
    int         current_iteration;
    double      current_temperature;
    double      best_cost;
    Candidate   *cand;

    bool        alter_candidate();
    
    double      __probability_of_accepting(double new_cost);
    void        __reset_path(Vehicle *car_pointer, std::vector<int>& refused_index);
    void        __chose_new_requests(const std::vector<int>& refused_index);
    void        __build_new_path(Vehicle *v1, Vehicle *v2);

public:
    SimulatedAnnealingOptimization();
    ~SimulatedAnnealingOptimization(){};
    Candidate* run(Candidate *ant);
};
