#include "config_loader.hpp"
#include "constants.hpp"
#include <fstream>
#include <sstream>

void ConfigLoader::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        if (std::getline(iss, key, ' ')) {
            double value;
            if (iss >> value) {
                if (key == "$max_t") Config::MAX_T = value;
                else if (key == "$max_rays") Config::MAX_RAYS = value;
                else if (key == "$min_energy_density") Config::MIN_ENERGY_DENSITY = value;
                else if (key == "$min_eps") Config::MIN_EPS = value;
            }
        }
    }
}