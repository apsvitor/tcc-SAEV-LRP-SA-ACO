#include "simulated_annealing.h"
#include <fstream>

#define INITIAL_T 100.0
#define COOLING_FACTOR 0.95
#define NEIGHBOR_REDUCTION_FACTOR 0.0005
#define MIN_T 1.0
#define MAX_ITERATIONS 50
#define MAX_NEIGHBORS_ITERATIONS 50

int main() {
    // Load input data
    int                     num_of_stations;
    int                     num_of_requests;
    std::queue<Request>     all_requests;
    std::vector<Station>    all_stations;
    
    std::ifstream inp("entrada.txt");
    
    // Load stations
    int x_station, y_station;
    inp >> num_of_stations;
    for (int station=1; station <= num_of_stations; station++) {
        inp >> x_station >> y_station;
        all_stations.push_back(Station(Point(x_station, y_station), station));
    }

    // Load Requests
    int x_origin, y_origin, x_destination, y_destination, pickup_time;
    inp >> num_of_requests;
    for (int request=1; request<=num_of_requests; request++) {
        inp >> x_origin >> y_origin >> x_destination >> y_destination >> pickup_time;
        all_requests.push(
            Request(Point(x_origin, y_origin), 
                    Point(x_destination, y_destination), 
                    pickup_time, 
                    request
            )
        );
    }
    inp.close();

    // PRINT REQUESTS
    // for (auto request: all_requests) {
    //         Point o = request.get_origin();
    //         Point d = request.get_destination();
    //         int   t = request.get_pickup_time();
    //         std::cout << "request: O(" << o.get_x() << ", " << o.get_y() << ") | D(" 
    //                   << d.get_x() << ", " << d.get_y() << ") | T = " << t << std::endl;
    // }

    std::cout << "STARTING SIMULATED ANNEALING SOLUTION FOR LRP-SAEV\n";
    SimulatedAnnealing SimAnn(
        all_stations,
        all_requests,
        INITIAL_T, 
        COOLING_FACTOR, 
        NEIGHBOR_REDUCTION_FACTOR, 
        MIN_T, 
        MAX_ITERATIONS, 
        MAX_NEIGHBORS_ITERATIONS
    );
    SimAnn.run();
}