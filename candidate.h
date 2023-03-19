#pragma once

#include "vehicle.h"


class Candidate {
private:
    std::vector<Vehicle>    all_vehicles;
    std::queue<Request>     all_requests;
    std::vector<Station>    all_stations;
    double                  candidate_cost;

public:
    Candidate(std::queue<Request> all_requests,
              std::vector<Station> all_stations);
    void generate_candidate();

    std::vector<Vehicle> get_all_vehicles();
    std::queue<Request> get_all_requests();
    std::vector<Station> get_all_stations();
    double get_candidate_cost();
    void set_candidate_cost(double cost);
    bool replace_vehicle();
    bool steal_request();
};