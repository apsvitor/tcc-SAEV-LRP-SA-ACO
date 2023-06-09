#include <fstream>
#include "aco.h"


int main() {
    // std::ifstream inp("entrada.txt");
    std::ifstream inp("./tests/S2_A6_T2.txt");

    int num_stations, num_requests;
    inp >> num_stations;

    std::vector<Vertex*> vertices_list;
    
    for (int id=0; id < num_stations; id++) {
        double x_station, y_station;
        inp >> x_station >> y_station;
        Vertex *station_v = new Station(Point(x_station, y_station), id);
        vertices_list.push_back(station_v);
    }

    inp >> num_requests;
    for (int id=0; id < num_requests; id++) {
        double x_start, y_start, x_end, y_end;
        int pickup_time;
        inp >> x_start >> y_start >> x_end >> y_end >> pickup_time;
        Vertex *request_v = new Request(
            Point(x_start, y_start), Point(x_end, y_end), pickup_time, id
        );
        vertices_list.push_back(request_v);
    }

    AntColonyOptimization ACO = AntColonyOptimization(vertices_list);
    ACO.run();
    inp.close();
}
