#include <random>

namespace random_gen{    
    template<typename T>
    T random_int(T range_from, T range_to) {
        std::random_device                  rand_dev;
        std::mt19937                        generator(rand_dev());
        std::uniform_int_distribution<T>    distr(range_from, range_to);
        return distr(generator);
    }

    template<typename T>
    T random_float(T range_from, T range_to) {
        std::random_device                  rand_dev;
        std::mt19937                        generator(rand_dev());
        std::uniform_real_distribution<T>   distr(range_from, range_to);
        return distr(generator);
    }
}


enum TripType {
    REQUEST_STATION,
    STATION_REQUEST,
    REQUEST_REQUEST,
    REQUEST_STATION_REQUEST,
    STATION_STOP,
    REQUEST_STOP
};

class Trip {
public:
    bool        is_feasible;
    double      energy_total;
    int         time_total;
    int         A; 
    int         B;
    int         C;
    int         D;
    TripType    trip_type;

    Trip() {};

    Trip(bool is_feasible, double energy_total, int time_total, 
         int A, int B, int C, int D, TripType trip_type){
        this->is_feasible   = is_feasible;
        this->energy_total  = energy_total;
        this->time_total    = time_total;
        this->A             = A;
        this->B             = B;
        this->C             = C;
        this->D             = D;
        this->trip_type     = trip_type;
    };
};