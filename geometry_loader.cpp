#include "geometry_loader.hpp"
#include "mpvector.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include "ray.hpp"
#include "lenses.hpp"
#include "mirror.hpp"

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
        if (line.find("$convexLens") == 0) {
            try {
                ConvexLens lens = parseConvexLensLine(line);
                devices.push_back(std::make_unique<ConvexLens>(lens.origin, lens.radius, lens.refractiveIndex, lens.height));
            } catch (const std::exception& e) {
                std::cerr << "Error parsing Convex: " << e.what() << "\n";
                continue;
            }
        }

        if (line.find("$sphericalLens") == 0) {
            try {
                SphericalLens lens = parseSphericalLensLine(line);
                devices.push_back(std::make_unique<SphericalLens>(lens.origin, lens.radius, lens.refractiveIndex));
            } catch (const std::exception& e) {
                std::cerr << "Error parsing SphericalLens: " << e.what() << "\n";
                continue;
            }
        }
        if (line.find("$parallelRays") == 0) {
            try {
                std::vector<Ray> raysToAdd = parseParallelRays(line);
                // std::cout << " AAAA " << raysToAdd.size() << "\n";
                rays.insert(rays.end(),
                raysToAdd.begin(),
                raysToAdd.begin() + std::min(raysToAdd.size(), Config::MAX_RAYS - rays.size()));
            } catch (const std::exception& e) {
                std::cerr << "Error parsing Mirror: " << e.what() << "\n";
                continue;
            }
        }
        
        if (line.find("$mirror") == 0) {
            try {
                Mirror mirror = parseMirror(line);
                devices.push_back(std::make_unique<Mirror>(mirror.origin, mirror.sideA, mirror.sideB, mirror.reflectance));
            } catch (const std::exception& e) {
                std::cerr << "Error parsing Mirror: " << e.what() << "\n";
                continue;
            }
        }
        if (line.find("$parabolicMirror") == 0) {
            try { 
                ParabolicMirror pm = parseParabolicMirror(line);
                devices.push_back(std::make_unique<ParabolicMirror>(pm.origin, pm.height, pm.curvature, pm.reflectance));
            } catch (const std::exception& e) {
                std::cerr << "Error parsing ParabolicMirror: " << e.what() << "\n";
                continue;
            }
        }
    }
}

GeometryObject GeometryLoader::parseLine (const std::string& line) {
    GeometryObject go{};

    size_t first_space = line.find(' ');
    std::istringstream iss(line.substr(first_space + 1));
    std::string token;

     while (iss >> token) {
        size_t eq_pos = token.find('=');
        if (eq_pos == std::string::npos) continue;

        std::string key = token.substr(0, eq_pos);
        std::string value_str = token.substr(eq_pos + 1);

        static const auto actions = std::unordered_map<std::string, std::function<void()>>{
            {"o",      [&] { go.origin = parseVector(value_str); }},
            {"d",      [&] { go.direction = parseVector(value_str); }},
            {"e",      [&] { go.energyDensity = std::stod(value_str); }},
            {"n",      [&] { go.refractiveIndex = std::stod(value_str); }},
            {"lambda", [&] { go.wavelength = std::stod(value_str); }},
            {"first",  [&] { go.first = parseVector(value_str); }},
            {"last",   [&] { go.last = parseVector(value_str); }},
            {"steps",  [&] { go.steps = std::stod(value_str); }},
            {"r",      [&] { go.r = std::stod(value_str); }},
            {"a",      [&] { go.a = parseVector(value_str); }},
            {"b",      [&] { go.b = parseVector(value_str); }},
            {"reflectance", [&] { go.reflectance = std::stod(value_str); }}
        };

        if (auto it = actions.find(key); it != actions.end()) {
            it->second();  // Execute action
        } else {
            std::cerr << "Unknown key: " << key << "\n";
        }
    }
    return go;
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

std::vector<Ray> GeometryLoader::parseParallelRays (const std::string& line) {
    std::istringstream iss(line.substr(13));  // Skip "$parallelRays"
    std::string token;
    Vector direction;
    Vector first;
    Vector last;
    int steps = 1;
    double energyDensity = 0;
    double refractiveIndex = 1;
    double wavelength = 550e-9;

    while (iss >> token) {
        size_t eq_pos = token.find('=');
        if (eq_pos == std::string::npos) continue;

        std::string key = token.substr(0, eq_pos);
        std::string value_str = token.substr(eq_pos + 1);

        if (key == "direction") {
            direction = parseVector(value_str);
        } else if (key == "first") {
            first = parseVector(value_str);
        } else if (key == "last") {
            last = parseVector(value_str);
        } else if (key == "steps") {
            steps = std::stod(value_str);  
        } else if (key == "e") {
            energyDensity = std::stod(value_str);        
        } else if (key == "n") {
            refractiveIndex = std::stod(value_str);
        } else if (key == "lambda") {
            wavelength = std::stod(value_str);
        }
    }

    if (direction.magnitude() * first.magnitude() * last.magnitude() == 0) {
        throw line;
    }

    return makeParallelRays(direction, first, last, steps,
        energyDensity, refractiveIndex, wavelength);
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

ConvexLens GeometryLoader::parseConvexLensLine (const std::string& line) {
    std::istringstream iss(line.substr(11));  // Skip "$convexLens"
    std::string token;
    Vector origin_, height_;
    double radius_, n_;

    while (iss >> token) {
        size_t eq_pos = token.find('=');
        if (eq_pos == std::string::npos) continue;

        std::string key = token.substr(0, eq_pos);
        std::string value_str = token.substr(eq_pos + 1);

        if (key == "o") {
            origin_ = parseVector(value_str);
        } else if (key == "r") {
            radius_ = std::stod(value_str);
        } else if (key == "n") {
            n_ = std::stod(value_str);
        } else if (key == "h") {
            height_ = parseVector(value_str);
        }
    }

    return ConvexLens(origin_, radius_, n_, height_);
}    

Mirror GeometryLoader::parseMirror (const std::string& line) {
    Mirror mirror{};
    std::istringstream iss(line.substr(7));  // Skip "$mirror"
    std::string token;

    while (iss >> token) {
        size_t eq_pos = token.find('=');
        if (eq_pos == std::string::npos) continue;

        std::string key = token.substr(0, eq_pos);
        std::string value_str = token.substr(eq_pos + 1);

        if (key == "o") {
            mirror.origin = parseVector(value_str);
        } else if (key == "a") {
            mirror.sideA = parseVector(value_str);
        } else if (key == "b") {
            mirror.sideB = parseVector(value_str);
        } else if (key == "reflectance") {
            mirror.reflectance = std::stod(value_str);
            mirror.transmittance = 1 - mirror.reflectance;
        }
    }

    if (mirror.sideA.cross(mirror.sideB).magnitude() == 0) { // also catches uninitialized
        throw line;
    }

    mirror.surfaceNormal = mirror.sideA.cross(mirror.sideB).normalized();

    return mirror;
}

ParabolicMirror GeometryLoader::parseParabolicMirror (const std::string& line) {
    ParabolicMirror pm{};
    std::istringstream iss(line.substr(16));  // Skip "$parabolicMirror"
    std::string token;

    while (iss >> token) {
        size_t eq_pos = token.find('=');
        if (eq_pos == std::string::npos) continue;

        std::string key = token.substr(0, eq_pos);
        std::string value_str = token.substr(eq_pos + 1);

        if (key == "o") {
            pm.origin = parseVector(value_str);
        } else if (key == "h") {
            pm.height = parseVector(value_str);
        } else if (key == "c") {
            pm.curvature = std::stod(value_str);
        } else if (key == "reflectance") {
            pm.reflectance = std::stod(value_str);
        }
    }

    return pm;
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