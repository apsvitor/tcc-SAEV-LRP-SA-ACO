#pragma once

#include "vehicle.h"
#include "vertex.h"
#include "utils.h"
#include <map>
#include <iostream>

class Candidate {
private:
    std::vector<Vehicle>    all_vehicles;   // solution per-say
    std::vector<Vertex*>    vertices_list;  // input

    std::vector<int>        s_ind;          // station indices
    std::vector<int>        r_ind;          // request indices

    int                     num_requests;   // probably useless
    int                     num_stations;   // probably useless
    int                     num_vertices;   // probably useless

    double                  candidate_cost;

    int         __station_randomizer();
    Vehicle*    __generate_new_vehicle(int v_index);
    Vertex*     __find_a_station_to_stop(Vehicle *car_pointer);
    Vertex*     __choose_next_edge(
                    std::map<pkey, float> &pheromone_matrix,
                    Vertex *current_v);
    void        __path_builder(
                    std::map<pkey, float> &pheromone_matrix,
                    Vehicle *car_pointer);

    double      __calculate_candidate_cost();

public:
    Candidate(
            std::vector<Vertex *>   vertices_list,
            std::vector<int>        s_ind,
            std::vector<int>        r_ind
    );
    void generate_candidate(std::map <pkey, float> &pheromone_matrix);

    std::vector<Vehicle> get_all_vehicles();
    double get_candidate_cost();
};
