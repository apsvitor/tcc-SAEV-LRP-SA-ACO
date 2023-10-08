#include <fstream>
#include <time.h>
#include "aco.h"

void solve();

int main() {
    solve();
}



void solve(){
    // Test sets
    // NOTE: could be replaced by a directory sweep later
    std::string instances[] = {
        // "entrada",
        "S2_A4_T1",
        "S2_A8_T1",
        "S2_A12_T1",
        "S2_A16_T1",
        "S2_A20_T1",
        "S2_A25_T1",
        "S2_A30_T1",
        "S2_A50_T1",
        "S2_A75_T1",
        "S2_A100_T1"
    };
    std::string PROBLEM_TYPE = "RPST";
    int NUM_OF_INSTANCE_TESTS = 20;
    int NUM_OF_INSTANCES = 10;          // instances.size()

    for (int instance=1; instance<=NUM_OF_INSTANCES; instance++) {
        std::string instance_name = instances[instance-1];

        std::string output_file_path = "./results/" + PROBLEM_TYPE + "/" + instance_name + ".csv";
        std::ofstream out(output_file_path);
        out << "indice;instancia;custo ACO-SA;tempo(s)\n";

        for (int test=1; test<=NUM_OF_INSTANCE_TESTS; test++) {
            std::string input_file_path = "./tests/" + instance_name + ".txt";
            std::ifstream inp(input_file_path);
            std::vector<Vertex*> vertices_list;
            int num_stations, num_requests;
            inp >> num_stations;
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
                Vertex *request_v = new Request(Point(x_start, y_start),
                                                Point(x_end, y_end),
                                                pickup_time, id, 0);

                for (int i=1; i<num_stations; i++){
                    if  (static_cast<Request*>(request_v)->destination.get_distance(vertices_list[i]->p_xy) < 
                         static_cast<Request*>(request_v)->destination.get_distance(vertices_list[static_cast<Request*>(request_v)->closest_station]->p_xy))
                         static_cast<Request*>(request_v)->closest_station = i;
                }

                vertices_list.push_back(request_v);
            }
        
            inp.close();

            clock_t tStart = clock();
            out << test << ';' << instance_name << ';';
            std::cout << num_requests << " " << vertices_list.size() << std::endl;

            AntColonyOptimization ACO = AntColonyOptimization(vertices_list);
            ACO.run();
            out << ACO.get_best_candidate_cost() << ';';
            out << (double)(clock() - tStart)/CLOCKS_PER_SEC << std::endl;

            for (unsigned int vert=0; vert<vertices_list.size(); vert++)
                delete vertices_list[vert];
        }
        out.close();
    }
}