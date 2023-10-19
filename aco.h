#pragma once
#include "candidate.h"
#include "simulated_annealing.h"
#include "vertex.h"
#include <map>
#include <iostream>

class AntColonyOptimization {
public:
    // additional attributes
    // This pheromone matrix will be a representation of every possible edge
    // between vertices and each pair (i,j) contains the current pheromone value.
    std::vector <Vertex*>   vertices_list;
    std::map <pkey, float>  pheromone_matrix;

    std::vector<int>        s_ind;              // station indices
    std::vector<int>        r_ind;              // request indices

    Candidate               *global_best;

   // auxiliary methods
    // std::vector<Candidate*> _ant_builder(std::vector<Candidate*>);
    // void  _ant_builder();
    Candidate*              ant_builder();
    void                    _create_pheromone_matrix();
    // void                    _update_pheromone_trail();
    void                    update_pheromone_trail(Candidate *ant);

    AntColonyOptimization(std::vector<Vertex*> vertices_list);
    double get_best_candidate_cost();
    void run();
};