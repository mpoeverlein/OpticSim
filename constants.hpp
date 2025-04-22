#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <limits>

const double MAX_T = 10; // maximum time
const double MIN_EPS = 1e-10; // minimum distance
const int MAX_RAYS = 1000;
const double MIN_ENERGY_DENSITY = 1.e-2; // minimum energy density for rays to be simulated

const double Inf = std::numeric_limits<double>::infinity();
const double negative_Inf= Inf*-1;


#endif /* CONSTANTS_HPP */