#include <iostream>
#include <fstream>
#include <random>
#include "../point.h"

int main(int argc, char *argv[]) {
    int     vertices = 5;
    int     map_radius = 40;
    if  (argc > 1) {
        vertices = atoi(argv[1]);
        map_radius = atoi(argv[2]);
    }

    double  city_distances[vertices][vertices];
    Point*  points = new Point[vertices];
    
    std::ofstream out_points("points.txt");
    std::ofstream out_distances("distances.txt");

    // random number generator
    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());
    std::uniform_int_distribution<int> distr(-map_radius, map_radius);
    
    // generate random points
    int x, y;
    for (int i = 0; i < vertices; i++) {
        x = distr(generator);
        y = distr(generator);
        points[i] = Point(x,y);
        out_points << x << ' ' << y << '\n';
    }
    out_points.close();
    
    // calculate distances between points
    for (int i = 0; i < vertices; i++) {
        for (int j = 0; j < vertices; j++) {
            if  (i==j)
                city_distances[i][j] = 0;
            else {
                city_distances[i][j] = points[i].get_distance(points[j]);
                city_distances[j][i] = city_distances[j][i];
            }
        }
    }

    for (int i = 0; i <vertices; i++) {
        for (int j = 0; j <vertices; j++)
            out_distances << city_distances[i][j] << ' ';
        out_distances << std::endl;
    }

    out_distances.close();

    // deallocate points
    delete [] points;

}