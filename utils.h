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
