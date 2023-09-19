#include "candidate.h"

Candidate::Candidate() {
    // dummy candidate
    this->candidate_cost = INT32_MAX;
}

Candidate::Candidate(
        std::vector<Vertex*> vertices_list,
        std::vector<int>     s_ind,
        std::vector<int>     r_ind){
    // Indices will serve as reference for RNG
    this->vertices_list = vertices_list;
    this->s_ind         = s_ind;
    this->r_ind         = r_ind;
    this->num_stations  = s_ind.size();
    this->remaining_requests  = r_ind.size();
    this->ignored_requests = std::floor(this->remaining_requests * (1.0 - request_c::MIN_REQUESTS_DONE));
}

Candidate::~Candidate() {
    for (int i=0; i<this->all_vehicles.size();i++){
        delete this->all_vehicles[i];
    }
}

int Candidate::__station_randomizer() {
    // returns the index for the station in the vertices list
    int s_index =  random_gen::random_int(0, this->num_stations-1);
    return this->s_ind[s_index];
}


Vehicle* Candidate::__generate_new_vehicle(int v_index) {
    int station_index = __station_randomizer();
    static_cast<Station*>(this->vertices_list[station_index])->is_used += 1;
    Vehicle* car_pointer = new Vehicle(station_index, v_index);

    return car_pointer;
}


void Candidate::generate_candidate(std::map <pkey, float> &pheromone_matrix) {
    int v_index=0;
    Vehicle *car_pointer;
    this->remaining_requests = this->r_ind.size();
    for (auto &v: this->vertices_list) {
        if  (v->vertex_type == 'r')
            static_cast<Request*>(v)->is_done = false;
        else
            static_cast<Station*>(v)->is_used = 0;
    }

    // Concludes candidate generation whenever the minimum threshold is met.
    while(this->remaining_requests != this->ignored_requests) {
        // acquire a vehicle
        car_pointer = __generate_new_vehicle(v_index++);
        // build the path for the vehicle
        bool keep_working = true;
        clock_t tStart = clock();
        int cont=0;
        while(keep_working) {
            keep_working = path_builder(pheromone_matrix, car_pointer);
            cont++;
        }
        std::cout << "\t\tVehicle [" << v_index << "] [" << cont << "] --> " << (double)(clock() - tStart)/CLOCKS_PER_SEC << std::endl;
        // add vehicle to the solution
        this->all_vehicles.push_back(car_pointer);
    }

    // calculate the cost function for the candidate
    this->candidate_cost = __calculate_cost();
}

double Candidate::__calculate_cost() {
    double station_cost     = 0,
           vehicle_cost     = 0,
           trip_cost        = 0,
           penalty_cost     = 0;

    // for each vehicle, adds the cost of acquisition
    vehicle_cost = this->all_vehicles.size() * vehicle_c::COST_PER_VEHICLE;

    // checks which stations were used and adds up their cost
    for (auto station_index: this->s_ind)
        if  (static_cast<Station*>(this->vertices_list[station_index])->is_used)
            station_cost += station_c::COST_PER_STATION;

    // for each vehicle, calculate the trip cost
    for (auto vehicle: this->all_vehicles)
        trip_cost += (vehicle->vehicle_path.size()-1) * request_c::COST_PER_TRIP;
    
    // for each unanswered request apply a penalty
    penalty_cost = this->ignored_requests * request_c::UNSERVED_PENALTY;

    return (station_cost + vehicle_cost + trip_cost + penalty_cost);
}


std::vector<Vehicle*> Candidate::get_all_vehicles(){
    return this->all_vehicles;
}

double Candidate::get_candidate_cost() {
    return this->candidate_cost;
}

void Candidate::change_vehicle(int index, Vehicle *new_vehicle, int new_cost){
    this->all_vehicles[index] = new_vehicle;
    this->candidate_cost = new_cost;
}



bool Candidate::validate_path(std::vector<Vertex> path) {
    // simulates a situation given a full path
    // vehicle starts at path[0], time=0, battery=100%
    int time_of_vehicle = 0;
    double battery      = vehicle_c::MAX_BATTERY;

    bool    is_on_time = true, 
            has_energy = true;
    for (int i=0; i<path.size()-1; i++) {
        // move type: station -> request
        if  (path[i].vertex_type == 's' && path[i+1].vertex_id == 'r') {
            // the vehicle IS RECHARGING
            double distance_to_origin   = path[i].p_xy.get_distance(path[i+1].p_xy);
            double distance_of_request  = static_cast<Request*>(&(path[i+1]))->request_distance;

            int request_starting_time   = static_cast<Request*>(&(path[i+1]))->pickup_time;
            int time_to_finish_request  = std::ceil(distance_of_request/vehicle_c::MEAN_VELOCITY);
            int time_to_reach_origin    = std::ceil(distance_to_origin/vehicle_c::MEAN_VELOCITY);
            int total_time_cost         = time_of_vehicle + time_to_reach_origin;

            double total_energy_cost    = (distance_to_origin + distance_of_request) * vehicle_c::CONSUMPTION_RATE;
            double missing_energy       = vehicle_c::MAX_BATTERY - battery;

            // if the problem allows partial recharge
            if  (problem_type::IS_PARTIAL_RECHARGE) {
                // calculate the minimum energy required to complete the request
                double  min_energy      = vehicle_c::MIN_BATTERY_LEVEL * vehicle_c::MAX_BATTERY,
                        request_energy  = (distance_to_origin + distance_of_request) * vehicle_c::CONSUMPTION_RATE,
                        min_recharge    = min_energy + request_energy - battery;

                // calculate the time necessary to get that minimum charge
                if  (min_recharge > 0) {
                    int min_recharge_time   = std::ceil(min_recharge / vehicle_c::CHARGING_RATE);
                    total_time_cost         += min_recharge_time;

                    // check if its possible to charge more
                    int max_charging_time   = request_starting_time - (time_of_vehicle + time_to_reach_origin);
                    double max_recharge     = std::min(max_charging_time*vehicle_c::CHARGING_RATE, missing_energy);
                    int time_spent_charging = std::ceil(max_recharge / vehicle_c::CHARGING_RATE);

                    total_time_cost += std::max(time_spent_charging, min_recharge_time);
                    battery         += std::max(max_recharge, min_recharge);
                }
            }
            // the problem demands full recharge
            else {
                double missing_battery = vehicle_c::MAX_BATTERY - battery;
                int time_to_fully_charge = missing_battery / vehicle_c::CHARGING_RATE;
                battery = vehicle_c::MAX_BATTERY;

                // time to fully recharge
                total_time_cost = std::max(total_time_cost + time_to_fully_charge, request_starting_time);
            }
            // time feasibility
            if  (total_time_cost > request_starting_time + request_c::LATENESS_EPS)
                return false;

            // energy feasibility is just a check since it passed the time check with min_recharge
            if  (battery - total_energy_cost < vehicle_c::MIN_BATTERY_LEVEL * vehicle_c::MAX_BATTERY)
                return false;
            // update the vehicle simulation
            time_of_vehicle = total_time_cost;
            battery -= total_energy_cost;
        }
        // move type: request -> request
        else if (path[i].vertex_type == 'r' && path[i+1].vertex_type == 'r') {
            int starting_time = static_cast<Request*>(&(path[i+1]))->pickup_time;
            double distance_of_request = static_cast<Request*>(&(path[i+1]))->request_distance;
            double distance_to_origin = path[i].p_xy.get_distance(path[i+1].p_xy);
            int time_to_reach_origin = std::ceil(distance_to_origin / vehicle_c::MEAN_VELOCITY);
            int time_to_finish_request = std::ceil(distance_of_request / vehicle_c::MEAN_VELOCITY);

            // time feasibility
            if  (time_to_reach_origin + time_of_vehicle > starting_time + request_c::LATENESS_EPS)
                return false;
            
            // energy feasibility
            double energy_cost = (distance_to_origin + distance_of_request) * vehicle_c::CONSUMPTION_RATE;
            if  (battery - energy_cost < vehicle_c::MIN_BATTERY_LEVEL * vehicle_c::MAX_BATTERY)
                return false;

            // updates the vehicle simulation
            time_of_vehicle = std::max(starting_time, time_to_reach_origin + time_of_vehicle) + time_to_finish_request;
            battery -= energy_cost;
        }
        // move type: request -> station
        else if (path[i].vertex_type == 'r' && path[i+1].vertex_type == 's') {
            // doesn't require time feasibility check
            double distance_to_station = path[i].p_xy.get_distance(path[i+1].p_xy);
            double energy_cost = distance_to_station * vehicle_c::CONSUMPTION_RATE;
            int time_to_reach_station = std::ceil(distance_to_station / vehicle_c::MEAN_VELOCITY);

            // energy feasibility
            if  (battery - energy_cost < vehicle_c::MIN_BATTERY_LEVEL * vehicle_c::MAX_BATTERY)
                return false;
            
            // updates the vehicle simulation
            time_of_vehicle += time_to_reach_station;
            battery -= energy_cost;
        }
        else if (path[i].vertex_type == 's' && path[i+1].vertex_type == 's') {
            return false;
        }
    }
    return (is_on_time & has_energy);
}

bool Candidate::path_builder(std::map<pkey, float> &pheromone_matrix,
                  Vehicle *car_pointer){
    // build a path to a given vehicle
    try {
        Trip trip_chosen = choose_next_trip(pheromone_matrix, car_pointer);
        update_vehicle(car_pointer, trip_chosen);
        return trip_chosen.is_feasible;
    }
    catch (const std::exception &error) {
        std::cout << error.what() << std::endl;
    }
    return false;
}

Trip Candidate::choose_next_trip(std::map<pkey, float> &pheromone_matrix,
                                    Vehicle *car_pointer) {
    // choose a valid node between all of current_v's neighbors
    Vertex *current_v = this->vertices_list[car_pointer->current_vertex];
    std::vector<Vertex*> adj_list = current_v->adj_list;

    // cumulative sum to represent the weights of the edges
    std::vector<double> cumulative_sum;

    // pheromone matrix origin reference
    pci id_i = pci(current_v->vertex_type, current_v->vertex_id);

    // auxiliary ACO variables
    double sum_components_value = 0, heuristic_value, pheromone_value;

    // trip pool
    std::vector<Trip> feasible_trips;

    // search through all of current_v's neighbors and find eligible moves
    for (auto neighbor_v: adj_list) {
        // if it's a request that's already done -> invalid node
        if  (!((neighbor_v->vertex_type == 'r') &&
              (static_cast<Request*>(neighbor_v)->is_done == true))) {
            Trip trip;
            try {
                trip = this->is_feasible(car_pointer, neighbor_v);
            }
            catch (const std::exception &error){
                std::cout << error.what() << std:: endl;
            }

            if  (trip.is_feasible) {
                // add trip to the pool
                feasible_trips.push_back(trip);

                // heuristic information
                heuristic_value = __calculate_heuristic_value(trip);

                // pheromone value
                pci id_j = pci(neighbor_v->vertex_type, neighbor_v->vertex_id);
                pheromone_value = pow(pheromone_matrix[pkey(id_i, id_j)], aco_c::ALPHA);

                sum_components_value += (pheromone_value * heuristic_value);
                // fix the value pushed into the cumulative sum to consider heuristic info and pheromone value
                cumulative_sum.push_back(pheromone_value * heuristic_value);
                if  (cumulative_sum.size() > 1)
                    cumulative_sum[cumulative_sum.size()-1] += cumulative_sum[cumulative_sum.size()-2];
            }
        }
    }

    // at a station and feasible_trips is empty, vehicle stays at such station.
    if  (current_v->vertex_type == 's' && feasible_trips.empty()) {
        return Trip(false, 0, 0, current_v->vertex_id, -1, -1, -1, STATION_STOP);
    }

    // at a request and feasible_trips size equals the number of stations, find a station to stop.
    if  (current_v->vertex_type == 'r' && feasible_trips.size() == this->num_stations) {
        for (int station: this->s_ind) {
            if  (static_cast<Station*>(this->vertices_list[station])->is_used) {
                // std::cout << this->vertices_list[station]->vertex_type << '_' << this->vertices_list[station]->vertex_id
                //           << "->" << current_v->vertex_type << '_' << current_v->vertex_id << std::endl;
                double dist_ab = current_v->p_xy.get_distance(this->vertices_list[station]->p_xy);
                double energy_total = car_pointer->current_battery - (dist_ab * vehicle_c::CONSUMPTION_RATE);
                int time_total = std::ceil(car_pointer->time_of_vehicle + (dist_ab / vehicle_c::MEAN_VELOCITY));
                // std::cout << "dist_ab: " << dist_ab << "  energy_total: " << energy_total << std::endl;
                return Trip(false, energy_total, time_total, current_v->vertex_id, station, -1, -1, REQUEST_STOP);
            }
        }
        throw std::runtime_error(
            std::string("[Choose Next Trip] Input issue: unable to find a feasible path from request ")
            + std::to_string(current_v->vertex_id) + std::string(" to any of the stations.")
        );
    }

    // finds out which edge will be randomly chosen according to its interval
    double max_sum = (cumulative_sum[cumulative_sum.size()-1]) / (sum_components_value);
    double random_probability = random_gen::random_float(0.0, 1.0) * max_sum;

    // TODO: implement a binary search for larger/denser instances
    for (int i=0; i<cumulative_sum.size(); i++)
        if  (cumulative_sum[i] / sum_components_value >= random_probability)
            return feasible_trips[i];
    

    throw std::logic_error(std::string("[Choose Next Trip] Should never have reached this point."));
}

Trip Candidate::is_feasible(Vehicle* car_pointer, Vertex* destination) {
    bool is_feasible = true;
    // given the vehicle's current state, determine trip feasibility
    Vertex* origin          = this->vertices_list[car_pointer->current_vertex];
    double energy_vehicle   = car_pointer->current_battery;
    double time_vehicle     = car_pointer->time_of_vehicle;
    double energy_max       = vehicle_c::MAX_BATTERY;
    double energy_min       = energy_min * vehicle_c::MIN_BATTERY_LEVEL;

    // Hypothetical station visit
    // A vehicle must be able to reach a station after finishing a request
    Vertex* station_hyp = this->vertices_list[this->__station_randomizer()];

    // trip information
    double dist_ab = 0, dist_bc = 0, dist_cd = 0, dist_ds = 0,
           energy_ab = 0, energy_bc = 0, energy_cd = 0, energy_ds = 0,
           time_ab = 0, time_bc = 0, time_cd = 0;
    
    double energy_vehicle_final, time_vehicle_final;
    int A, B, C, D;
    TripType trip_type;

    // Case 1: Request to Station - should always be true (SANITY CHECK)
    if  (origin->vertex_type == 'r' && destination->vertex_type == 's') {
        dist_ab = static_cast<Request*>(origin)->destination.get_distance(destination->p_xy);
        // dist_ab = origin->p_xy.get_distance(destination->p_xy);
        energy_ab = dist_ab * vehicle_c::CONSUMPTION_RATE;
        time_ab = dist_ab / vehicle_c::MEAN_VELOCITY;

        // check input. It is expected being able to reach any station from any request.
        if  (energy_vehicle - energy_ab < energy_min) {
            std::cout << "energy: " << energy_vehicle << "  energy_ab: " << energy_ab << "   energy_min " << energy_min << std::endl;
            throw std::runtime_error(
                std::string("[Is Feasible - Case 1] Input issue: unable to find a feasible path from request ")
                + std::to_string(origin->vertex_id) + std::string(" to any of the stations.")
            );
        }
        // trip information update
        energy_vehicle_final = energy_vehicle - energy_ab;
        time_vehicle_final = time_vehicle + time_ab;
        A = origin->vertex_id + this->num_stations;
        B = destination->vertex_id;
        C = D = -1;
        trip_type = REQUEST_STATION;
    }

    // Case 2: Station to Request:
    else if (origin->vertex_type == 's' && destination->vertex_type == 'r') {
        dist_ab     = origin->p_xy.get_distance(destination->p_xy);
        dist_bc     = static_cast<Request*>(destination)->request_distance;
        dist_ds     = static_cast<Request*>(destination)->destination.get_distance(station_hyp->p_xy);
        energy_ab   = dist_ab * vehicle_c::CONSUMPTION_RATE;
        energy_bc   = dist_bc * vehicle_c::CONSUMPTION_RATE;
        energy_ds   = dist_ds * vehicle_c::CONSUMPTION_RATE;
        time_ab     = dist_ab / vehicle_c::MEAN_VELOCITY;
        time_bc     = dist_bc / vehicle_c::MEAN_VELOCITY;

        int time_pickup = static_cast<Request*>(destination)->pickup_time;
        double time_spent_recharging;

        double dist_real_trip   = dist_ab + dist_bc;
        double dist_hyp_trip    = dist_real_trip + dist_ds;
        double time_real_trip   = dist_real_trip / vehicle_c::MEAN_VELOCITY;
        double energy_real_trip = dist_real_trip * vehicle_c::CONSUMPTION_RATE;
        double energy_hyp_trip  = dist_hyp_trip * vehicle_c::CONSUMPTION_RATE;

        // time available to recharge
        double time_until_trip_starts = time_pickup + request_c::LATENESS_EPS - time_vehicle - time_ab;
        double time_until_full_charge = (energy_max - energy_vehicle) / vehicle_c::CHARGING_RATE;

        if  (problem_type::IS_PARTIAL_RECHARGE)
            time_spent_recharging = std::min(time_until_full_charge, time_until_trip_starts);
        else
            time_spent_recharging = time_until_full_charge;
        
        if  (time_spent_recharging < 0 || // time-unfeasible even without recharging
            time_vehicle + time_spent_recharging + time_ab > time_pickup + request_c::LATENESS_EPS){
            is_feasible = false;
        }
        else {
            double energy_recharged = time_spent_recharging * vehicle_c::CHARGING_RATE;

            if  ((energy_vehicle + energy_recharged) - (energy_hyp_trip) < energy_min){
                is_feasible = false;
            }
            else {
                is_feasible = true;
                energy_vehicle_final = energy_vehicle + energy_recharged - energy_real_trip;
                time_vehicle_final = time_vehicle + time_spent_recharging + time_real_trip;
                A = origin->vertex_id;
                B = C = destination->vertex_id + this->num_stations;
                D = -1;
                trip_type = STATION_REQUEST;
            }
        }
    }
    // Case 3: Request to Request
    else if (origin->vertex_type == 'r' && destination->vertex_type == 'r') {
        // first we check if it is possible to complete the trip without recharging
        dist_ab     = origin->p_xy.get_distance(destination->p_xy);
        dist_bc     = static_cast<Request*>(destination)->request_distance;
        energy_ab   = dist_ab * vehicle_c::CONSUMPTION_RATE;
        energy_bc   = dist_bc * vehicle_c::CONSUMPTION_RATE;
        time_ab     = dist_ab / vehicle_c::MEAN_VELOCITY;
        time_bc     = dist_bc / vehicle_c::MEAN_VELOCITY;
        int time_pickup = static_cast<Request*>(destination)->pickup_time;

        // time feasibility
        if  (time_vehicle + time_ab > time_pickup + request_c::LATENESS_EPS) {
            is_feasible = false;
        }
        else {
            // energy feasibility
            if  (energy_vehicle - (energy_ab + energy_cd + energy_ds) < energy_min) {
                // Must Recharge -> recalculate route
                dist_ab = origin->p_xy.get_distance(station_hyp->p_xy);
                dist_bc = station_hyp->p_xy.get_distance(destination->p_xy);
                dist_cd = static_cast<Request*>(destination)->request_distance;
                dist_ds = static_cast<Request*>(destination)->destination.get_distance(station_hyp->p_xy);
                energy_ab = dist_ab * vehicle_c::CONSUMPTION_RATE;
                energy_bc = dist_bc * vehicle_c::CONSUMPTION_RATE;
                energy_cd = dist_cd * vehicle_c::CONSUMPTION_RATE;
                energy_ds = dist_ds * vehicle_c::CONSUMPTION_RATE;
                time_ab = dist_ab * vehicle_c::MEAN_VELOCITY;
                time_bc = dist_bc * vehicle_c::MEAN_VELOCITY;
                time_cd = dist_cd * vehicle_c::MEAN_VELOCITY;

                double energy_required = energy_ab + energy_bc + energy_cd + energy_ds;
                double energy_missing;
                if  (problem_type::IS_PARTIAL_RECHARGE)
                    energy_missing = abs(energy_vehicle - (energy_required + energy_min));
                else
                    energy_missing = abs((energy_vehicle - energy_ab) - vehicle_c::MAX_BATTERY);
                
                double time_recharging = energy_missing * vehicle_c::CHARGING_RATE;

                // new time feasibility
                if  (time_vehicle + time_ab + time_recharging + time_bc > time_pickup + request_c::LATENESS_EPS) {
                    is_feasible = false;
                }
                else {
                    is_feasible = true;
                    if  (problem_type::IS_PARTIAL_RECHARGE)
                        energy_vehicle_final = energy_vehicle + energy_missing - (energy_ab + energy_bc + energy_cd);
                    else  // the energy_ab is already accounted for in full recharge
                        energy_vehicle_final = energy_vehicle + energy_missing - (energy_bc + energy_cd);
                    time_vehicle_final = time_vehicle + time_ab + time_recharging + time_bc + time_cd;
                    A = origin->vertex_id + this->num_stations;
                    B = station_hyp->vertex_id;
                    C = D = destination->vertex_id + this->num_stations;
                    trip_type = REQUEST_STATION_REQUEST;
                }
            }
            else {
                // Recharge's not needed
                is_feasible = true;
                energy_vehicle_final = energy_vehicle - (energy_ab + energy_bc);
                time_vehicle_final = time_vehicle + time_ab + time_bc;
                A = origin->vertex_id + this->num_stations;
                B = C = destination->vertex_id + this->num_stations;
                D = -1;
                trip_type = REQUEST_REQUEST;
            }
        }
    }
    // Impossible move
    else {
        throw std::runtime_error(
            std::string("[Is Feasible] Input issue: an impossible move was found. ")
            + origin->vertex_type + '_' + std::to_string(origin->vertex_id) + " -> "
            + destination->vertex_type + '_' + std::to_string(destination->vertex_id)
        );
    }

    Trip trip = Trip(is_feasible, energy_vehicle_final, std::ceil(time_vehicle_final),
                     A, B, C, D, trip_type);
    return trip;
}


double Candidate::__calculate_heuristic_value(Trip trip) {
    double heuristic_value = 0.5;
    switch(trip.trip_type) {
        case REQUEST_STATION: {
            // - cost of 1 trip
            // - cost of recharging
            // - cost of opening a station (if it got open at the trip)
            if  (static_cast<Station*>(this->vertices_list[trip.B])->is_used == 0)
                heuristic_value = heuristic_info::OPEN_NEW_STATION;
            break;
        }
        case STATION_REQUEST: {
            // - cost of 2 trips
            // + bonus for answering a request
            break;
        }
        case REQUEST_REQUEST: {
            // - cost of 2 trips
            // + bonus for answering a request (follow-up answer)
            break;
        }
        case REQUEST_STATION_REQUEST: {
            // - cost of 3 trips
            // - cost of opening a station (if it got open at the trip)
            // - cost of recharging
            // + bonus for answering a request
            break;
        }
        default:
            throw std::runtime_error(std::string("[Calculate Heuristic Value] Input issue: invalid trip type."));
    }

    return pow(heuristic_value, aco_c::BETA);
}

void Candidate::update_vehicle(Vehicle *car_pointer, Trip trip) {
    // update vehicle battery
    car_pointer->current_battery = trip.energy_total;
    // update vehicle time
    car_pointer->time_of_vehicle = trip.time_total;
    // update vehicle and vertices according to trip type
    switch(trip.trip_type) {
        case REQUEST_STATION: {
            static_cast<Station*>(this->vertices_list[trip.B])->is_used += 1;
            car_pointer->current_vertex = trip.B;
            car_pointer->vehicle_path.push_back(trip.B);
            break;
        }
        case STATION_REQUEST: {
            this->remaining_requests--;
            static_cast<Request*>(this->vertices_list[trip.C])->is_done = true;
            car_pointer->current_vertex = trip.C;
            car_pointer->vehicle_path.push_back(trip.B);
            car_pointer->vehicle_path.push_back(trip.C);
            break;
        }
        case REQUEST_REQUEST: {
            this->remaining_requests--;
            static_cast<Request*>(this->vertices_list[trip.C])->is_done = true;
            car_pointer->current_vertex = trip.C;
            car_pointer->vehicle_path.push_back(trip.B);
            car_pointer->vehicle_path.push_back(trip.C);
            break;
        }
        case REQUEST_STATION_REQUEST: {
            this->remaining_requests--;
            static_cast<Station*>(this->vertices_list[trip.B])->is_used += 1;
            static_cast<Request*>(this->vertices_list[trip.D])->is_done = true;
            car_pointer->current_vertex = trip.D;
            car_pointer->vehicle_path.push_back(trip.B);
            car_pointer->vehicle_path.push_back(trip.C);
            car_pointer->vehicle_path.push_back(trip.D);
            break;
        }
        case STATION_STOP: {
            break;
        }
        case REQUEST_STOP: {
            car_pointer->current_vertex = trip.B;
            car_pointer->vehicle_path.push_back(trip.B);
            break;
        }
        default:
            throw std::runtime_error(std::string("[Update Vehicle] Input issue: invalid trip type."));
    }
}

