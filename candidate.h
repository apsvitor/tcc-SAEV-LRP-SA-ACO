#pragma once

#include "vehicle.h"
#include "vertex.h"
#include "utils.h"
#include <map>
#include <iostream>

class Candidate {
public:
    std::vector<Vehicle*>   all_vehicles;   // solution per-say

    std::vector<int>        s_ind;          // station indices
    std::vector<int>        r_ind;          // request indices

    unsigned int            remaining_requests;
    unsigned int            ignored_requests;
    int                     num_stations;
    int                     answer_count;

    double                  candidate_cost;

    std::vector<Vertex*>    vertices_list;  // input

    int         __station_randomizer();
    Vehicle*    __generate_new_vehicle(int v_index);

    Candidate();
    Candidate(
            std::vector<Vertex *>   vertices_list,
            std::vector<int>        s_ind,
            std::vector<int>        r_ind
    );
    ~Candidate();
    void generate_candidate(std::map <pkey, float> &pheromone_matrix);
    // test
    bool path_builder(std::map<pkey, float> &pheromone_matrix,
                      Vehicle *car_pointer);
    Trip choose_next_trip(std::map<pkey, float> &pheromone_matrix,
                             Vehicle *car_pointer);
    Trip is_feasible(Vehicle* car_pointer, Vertex* destination);
    void update_vehicle(Vehicle *car_pointer, Trip trip);
    double __calculate_heuristic_value(Trip trip);
    double __calculate_cost();
};
