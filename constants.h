
// SIMULATED ANNEALING PARAMETERS
namespace sa_c{
    const double    INITIAL_T                   = 100.0;
    const double    COOLING_FACTOR              = 0.95;
    const double    NEIGHBOR_REDUCTION_FACTOR   = 0.0005;
    const double    MIN_T                       = 1.0;
    const int       MAX_ITERATIONS              = 50;
    const int       MAX_NEIGHBORS_ITERATIONS    = 50;
}


// ANT COLONY OPTIMIZATION PARAMETERS
namespace aco_c{
    const double    ALPHA           = 0.5;
    const double    BETA            = 0.5;
    const double    RO              = 0.1;
    const double    BASE_PHEROMONE  = 0.5;
    const int       MAX_ANTS        = 10;
    const int       MAX_ITERATIONS  = 20;
}


// PROBLEM-RELATED CONSTANTS
// Vehicles
namespace vehicle_c {
    const double    MAX_BATTERY         = 15.0;     // KWh
    const double    CONSUMPTION_RATE    = 0.15;     // KWh/Km  
    const double    MEAN_VELOCITY       = 35.0/60;  // Km/min
    const double    CHARGING_RATE       = 0.4;      // KW/min
    const double    MIN_BATTERY_LEVEL   = 0.20;     // %
    const double    COST_PER_VEHICLE    = 137.0;    // $RMB
}
// Stations
namespace station_c {
    const double    COST_PER_STATION    = 2328.76;  // $RMB
}
// Requests
namespace request_c {
    const double    COST_PER_TRIP       = 19.37;    // $RMB
    const int       LATENESS_EPS        = 5;        // acceptable lateness (min)
}

// key pair for the pheromone matrix. Ex.: ('s', 1) -> Station 1
typedef std::pair<char, int> pci;
// key structure for the pheromone matrix. Ex.: (vertex_i, vertex_j)
typedef std::pair<pci, pci> pkey;
// key structure for the edge-choosing method
typedef std::pair<double, int> pdi;
