#pragma once

#include "vehicle.h"


class Candidate {
private:
    std::vector<Vehicle> all_vehicles;
    std::queue<Request> all_requests;
    std::vector<Station> all_stations;

public:
    Candidate(std::queue<Request> all_requests,
              std::vector<Station> all_stations);
    void generate_candidate();

    std::vector<Vehicle> get_all_vehicles();
    std::queue<Request> get_all_requests();
    std::vector<Station> get_all_stations();
};