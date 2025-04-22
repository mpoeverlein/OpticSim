#include "constants.hpp"

namespace Config {
    double VACUUM_REFRACTIVE_INDEX = 1.;
    double MAX_T = 10; // maximum time
    double MIN_EPS = 1e-10; // minimum distance
    int MAX_RAYS = 100;
    double MIN_ENERGY_DENSITY = 1.e-2; // minimum energy density for rays to be simulated
}
