#include "geometry_loader.hpp"
#include "mpvector.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include "ray.hpp"
#include "lenses.hpp"

void GeometryLoader::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }
    
    std::string line;
    
    while (std::getline(file, line)) {
        // comments denoted by "#"
        if (line.empty() || line[0] == '#') continue;

        if (line.find("$ray") == 0) {
            try {
                rays.push_back(parseRayLine(line));
            } catch (const std::exception& e) {
                std::cerr << "Error parsing ray: " << e.what() << "\n";
                continue;
            }
        }

        if (line.find("$sphericalLens") == 0) {
            try {
                // rays.push_back(parseRayLine(line));
                SphericalLens lens = parseSphericalLensLine(line);
                devices.push_back(std::make_unique<SphericalLens>(lens.origin, lens.radius, lens.refractiveIndex));
            } catch (const std::exception& e) {
                std::cerr << "Error parsing ray: " << e.what() << "\n";
                continue;
            }
        }
    }
}

Ray GeometryLoader::parseRayLine (const std::string& line) {
    Ray ray{};
    std::istringstream iss(line.substr(4));  // Skip "$ray"
    std::string token;

    while (iss >> token) {
        size_t eq_pos = token.find('=');
        if (eq_pos == std::string::npos) continue;

        std::string key = token.substr(0, eq_pos);
        std::string value_str = token.substr(eq_pos + 1);

        if (key == "o") {
            ray.origin = parseVector(value_str);
        } else if (key == "d") {
            ray.direction = parseVector(value_str);
        } else if (key == "e") {
            ray.energyDensity = std::stod(value_str);
        } else if (key == "n") {
            ray.refractiveIndex = std::stod(value_str);
        } else if (key == "lambda") {
            ray.wavelength = std::stod(value_str);
        }
    }

    if ((ray.direction.magnitude() == 0) || ray.energyDensity == 0) {
        throw line;
    }

    return ray;
}

SphericalLens GeometryLoader::parseSphericalLensLine (const std::string& line) {
    SphericalLens lens{};
    std::istringstream iss(line.substr(14));  // Skip "$sphericalLens"
    std::string token;

    while (iss >> token) {
        size_t eq_pos = token.find('=');
        if (eq_pos == std::string::npos) continue;

        std::string key = token.substr(0, eq_pos);
        std::string value_str = token.substr(eq_pos + 1);

        if (key == "o") {
            lens.origin = parseVector(value_str);
        } else if (key == "r") {
            lens.radius = std::stod(value_str);
        } else if (key == "n") {
            lens.refractiveIndex = std::stod(value_str);
        }
    }

    if ((lens.radius == 0) || lens.refractiveIndex == 1) {
        throw line;
    }

    return lens;
}


Vector GeometryLoader::parseVector(const std::string& str) {
    std::istringstream iss(str);
    char comma;
    double x, y, z;

    iss >> x >> comma >> y >> comma >> z;
    if (comma != ',' || iss.fail()) {
        throw std::runtime_error("Invalid vector format: " + str);
    }

    return Vector(x, y, z);
}