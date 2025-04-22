#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <limits>

namespace Config {
    extern double VACUUM_REFRACTIVE_INDEX;
    extern double MAX_T;
    extern double MIN_EPS;
    extern int MAX_RAYS;
    extern double MIN_ENERGY_DENSITY;
}



const double Inf = std::numeric_limits<double>::infinity();
const double negative_Inf= Inf*-1;


#endif /* CONSTANTS_HPP */