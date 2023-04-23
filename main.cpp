#include <fstream>
#include "aco.h"


int main() {
    std::ifstream inp("entrada.txt");
    int num_stations, num_requests;
    inp >> num_stations;
    
    std::vector<Vertex*> vertices_list;
    
    for (int id=0; id < num_stations; id++) {
        int x_station, y_station;
        inp >> x_station >> y_station;
        Vertex *station_v = new Station(Point(x_station, y_station), id);
        vertices_list.push_back(station_v);
    }

    inp >> num_requests;
    for (int id=0; id < num_requests; id++) {
        int x_start, y_start, x_end, y_end, pickup_time;
        inp >> x_start >> y_start >> x_end >> y_end >> pickup_time;
        Vertex *request_v = new Request(
            Point(x_start, y_start), Point(x_end, y_end), pickup_time, id
        );
        vertices_list.push_back(request_v);
    }

    AntColonyOptimization ACO = AntColonyOptimization(vertices_list);
    ACO.run();
    inp.close();

    // // Input data
    // int                     num_of_stations;
    // int                     num_of_requests;
    // std::vector<Request>    all_requests;
    // std::vector<Station>    all_stations;

    // std::ifstream inp("entrada.txt");
    
    // // Load stations
    // int x_station, y_station;
    // inp >> num_of_stations;
    // for (int station=1; station <= num_of_stations; station++) {
    //     inp >> x_station >> y_station;
    //     all_stations.push_back(Station(Point(x_station, y_station), station));
    // }

    // // Load Requests
    // int x_origin, y_origin, x_destination, y_destination, pickup_time;
    // inp >> num_of_requests;
    // for (int request=1; request<=num_of_requests; request++) {
    //     inp >> x_origin >> y_origin >> x_destination >> y_destination >> pickup_time;
    //     all_requests.push_back(
    //         Request(Point(x_origin, y_origin), 
    //                 Point(x_destination, y_destination), 
    //                 pickup_time, 
    //                 request
    //         )
    //     );
    // }
    // inp.close();
}