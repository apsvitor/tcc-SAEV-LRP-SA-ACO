/*
Main function for the TSP solution using
Simulated Annealing
*/
#include "simulated_annealing.h"
#include <fstream>

#define NUM_OF_CITIES 5
#define STARTING_CITY 0
#define INITIAL_T 1000.0
#define COOLING_FACTOR 0.95
#define NEIGHBOR_REDUCTION_FACTOR 0.0000005
#define MIN_T 1.0
#define MAX_ITERATIONS 1000
#define MAX_NEIGHBORS_ITERATIONS 500

int main() {
    // Load input data
    int                     num_of_stations;
    int                     num_of_requests;
    std::queue<Request>    all_requests;
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

    // PRINT REQUESTS
    // for (auto request: all_requests) {
    //         Point o = request.get_origin();
    //         Point d = request.get_destination();
    //         int   t = request.get_pickup_time();
    //         std::cout << "request: O(" << o.get_x() << ", " << o.get_y() << ") | D(" 
    //                   << d.get_x() << ", " << d.get_y() << ") | T = " << t << std::endl;
    // }


    
    Candidate c1(all_requests, all_stations);
    c1.generate_candidate();
    int i=0;
    std::vector<Vehicle> c1_vehicles = c1.get_all_vehicles();
    std::cout << "\n--------------------------------" << "Testing candidate\n"
              << "Qntd de veiculos: " << c1_vehicles.size() << "\n";
    
    for (auto vehicle: c1_vehicles) {
        std::vector<Request> vehicle_requests = vehicle.get_request_list();
        
        for (auto request: vehicle_requests) {
            Point o = request.get_origin();
            Point d = request.get_destination();
            int   t = request.get_pickup_time();
            std::cout << "Vehicle " << i << " request: O(" << o.get_x() << ", " << o.get_y() << ") | D(" 
                      << d.get_x() << ", " << d.get_y() << ") | T = " << t << std::endl;
        }
        ++i;
    }
    
    


    // // Load the city map
    // std::vector<Point> city_map;
    // std::ifstream inp_points("./input/points.txt");
    // int x,y;
    // while(inp_points >> x >> y)
    //     city_map.push_back(Point(x,y));
    // inp_points.close();
    // // Load the distances matrix
    // std::vector< std::vector<double> > distance_matrix;
    // std::ifstream inp_distance("./input/distances.txt");
    // double dist;
    // for (int i = 0; i < NUM_OF_CITIES; i++) {
    //     std::vector<double> aux;
    //     for (int j = 0; j < NUM_OF_CITIES; j++) {
    //         inp_distance >> dist;
    //         aux.push_back(dist);
    //     }
    //     distance_matrix.push_back(aux);
    // }
    // inp_distance.close();

    // std::cout << "STARTING SIMULATED ANNEALING SOLUTION FOR TSP\n";
    // SimulatedAnnealing SimAnn(
    //                       city_map, 
    //                       distance_matrix, 
    //                       STARTING_CITY, 
    //                       INITIAL_T, 
    //                       COOLING_FACTOR, 
    //                       NEIGHBOR_REDUCTION_FACTOR, 
    //                       MIN_T, 
    //                       MAX_ITERATIONS, 
    //                       MAX_NEIGHBORS_ITERATIONS
    //                     );
    // SimAnn.run();
}