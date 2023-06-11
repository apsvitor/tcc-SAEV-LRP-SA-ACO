#include <fstream>
#include <time.h>
#include "aco.h"

int main() {
    
    // std::ifstream inp("entrada.txt");
    // std::ifstream inp("./tests/S2_A6_T2.txt");
    // std::ifstream inp("./tests/S2_A10_T1.txt");
    std::string instance_name[] = {
        "S2_A4_T1",
        "S2_A5_T1",
        "S2_A6_T1",
        "S2_A6_T2",
        "S2_A7_T1",
        "S2_A7_T2",
        "S2_A8_T1",
        "S2_A10_T1",
        "S2_A12_T1",
        "S2_A16_T1",
        "S2_A20_T1",
        "S2_A25_T1",
        "S2_A30_T1",
        "S2_A30_T1",
        "S2_A50_T2",
        "S2_A75_T1",
        "S2_A100_T1",
        "S2_A100_T2"
    };
    std::ofstream out("tipo_rtst_round1_c.csv");
    out << "indice;instancia;custo ACO-SA;tempo(s)\n";
    for (int ind=16; ind<=18; ind++){
        clock_t tStart = clock();
        std::string test_file = "./tests/" + instance_name[ind-1] + ".txt";
        std::ifstream inp(test_file);
        out << ind << ';' << instance_name[ind-1] << ';';
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
        std::cout << num_requests << " " << vertices_list.size() << std::endl;

        AntColonyOptimization ACO = AntColonyOptimization(vertices_list);
        ACO.run();
        out << ACO.get_best_candidate_cost() << ';';
        inp.close();
        out << (double)(clock() - tStart)/CLOCKS_PER_SEC << std::endl;

    }
    out.close();
}
