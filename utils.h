#pragma once
#include <random>
#include <iostream>

namespace useful_prints{
    template<typename Candidate>
    void printSol(Candidate *cand, std::string local_info) {
        for (unsigned int i=0; i<cand->all_vehicles.size(); i++) {
            Vehicle *v = cand->all_vehicles[i];
            std::cout << "V[" << v->vehicle_id << "]: ";
            for (unsigned int j=0; j<v->vehicle_path.size(); j++) {
                Vertex *vtx = cand->vertices_list[v->vehicle_path[j]];
                if  (vtx->vertex_type == 'r') {
                    std::cout << "[r_";
                }
                else {
                    std::cout << "[s_";
                }
                std::cout << vtx->vertex_id << "] -> ";
            }
            std::cout << std::endl;
        }
        std::cout << local_info;
        for (unsigned int i=0; i<cand->vertices_list.size(); i++) {
            Vertex *vtx = cand->vertices_list[i];
            if  (vtx->vertex_type == 's'){
                std::cout << "s_" << vtx->vertex_id << ": " << static_cast<Station*>(vtx)->is_used << " | ";
            }
            else{
                break;
            }
        }
        std::cout << std::endl;
    }
}

namespace random_gen{    
    template<typename T>
    T random_int(T range_from, T range_to) {
        std::random_device                  rand_dev;
        std::mt19937                        generator(rand_dev());
        std::uniform_int_distribution<T>    distr(range_from, range_to);
        return distr(generator);
    }

    template<typename T>
    T random_float(T range_from, T range_to) {
        std::random_device                  rand_dev;
        std::mt19937                        generator(rand_dev());
        std::uniform_real_distribution<T>   distr(range_from, range_to);
        return distr(generator);
    }
}


enum TripType {
    REQUEST_STATION,
    STATION_REQUEST,
    REQUEST_REQUEST,
    REQUEST_STATION_REQUEST,
    STATION_STOP,
    REQUEST_STOP
};

class Trip {
public:
    bool        is_feasible;
    double      energy_total;
    int         time_total;
    int         A; 
    int         B;
    int         C;
    int         D;
    TripType    trip_type;

    Trip() {};

    Trip(bool is_feasible, double energy_total, int time_total, 
         int A, int B, int C, int D, TripType trip_type){
        this->is_feasible   = is_feasible;
        this->energy_total  = energy_total;
        this->time_total    = time_total;
        this->A             = A;
        this->B             = B;
        this->C             = C;
        this->D             = D;
        this->trip_type     = trip_type;
    };
};
