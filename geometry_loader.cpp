#include "geometry_loader.hpp"
#include "mpvector.hpp"

#include <fstream>
#include <sstream>
#include <iostream>
#include "ray.hpp"
#include "material.hpp"
#include "surfacegeometry.hpp"
#include "optdev.hpp"

std::unique_ptr<Material> GeometryObject::createMaterial () {
    if (refractiveIndex != 1) {
        return std::make_unique<NonDispersiveMaterial>(refractiveIndex);
    }
    if (material == "Water") {
        return std::make_unique<Water>(temperature);
    }
    std::cerr << "Unsupported material: " << material << "\n";
    return std::make_unique<NonDispersiveMaterial>(refractiveIndex);
}


void GeometryLoader::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << "\n"
            << "Geometry cannot be loaded.\n";
        return;
    }
    
    std::string line;
    
    while (std::getline(file, line)) {
        // comments denoted by "#"
        if (line.empty() || line[0] == '#') continue;

        if (line.find("$ray") == 0) {
            Ray ray{parseRayLine(line)};
            if (ray == Ray()) {
                std::cerr << "Error parsing ray from this line: " << line << "\n";
            } else {
                rays.push_back(ray);
            }
        } else if (line.find("$convexLens") == 0) {
            Lens lens = parseConvexLensLine(line);
            devices.push_back(std::make_unique<Lens>(std::move(lens)));
        }

        if (line.find("$concaveLens") == 0) {
            Lens lens = parseConcaveLensLine(line);
            devices.push_back(std::make_unique<Lens>(std::move(lens)));
        }

        if (line.find("$sphericalLens") == 0) {
            Lens lens = parseSphericalLensLine(line);
            devices.push_back(std::make_unique<Lens>(std::move(lens)));
        }

        if (line.find("$planoConvexLens") == 0) {
            Lens lens = parsePlanoConvexLensLine(line);
            devices.push_back(std::make_unique<Lens>(std::move(lens)));
        }

        if (line.find("$planoConcaveLens") == 0) {
            Lens lens = parsePlanoConcaveLensLine(line);
            devices.push_back(std::make_unique<Lens>(std::move(lens)));
        }


        if (line.find("$parallelRays") == 0) {
            try {
                std::vector<Ray> raysToAdd = parseParallelRays(line);
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

        if (key == "o") { go.origin = parseVector(value_str); }
        else if (key == "d") { go.direction = parseVector(value_str); }
        else if (key == "e") { go.energyDensity = std::stod(value_str); }
        else if (key == "n") { go.refractiveIndex = std::stod(value_str); }
        else if (key == "lambda") { go.wavelength = std::stod(value_str); }
        else if (key == "first") { go.first = parseVector(value_str); }
        else if (key == "last") { go.last = parseVector(value_str); }
        else if (key == "h") { go.height = parseVector(value_str); }
        else if (key == "steps") { go.steps = std::stod(value_str); }
        else if (key == "r") { go.r = std::stod(value_str); }
        else if (key == "a") { go.a = parseVector(value_str); }
        else if (key == "b") { go.b = parseVector(value_str); }
        else if (key == "c") { go.curvature = std::stod(value_str); }
        else if (key == "reflectance") { go.reflectance = std::stod(value_str); }
        else if (key == "material") { go.material = value_str; }
        else if (key == "temperature") { go.material = std::stod(value_str); }
        else if (key == "transverseR") { go.transverseRadius = std::stod(value_str); }
        else {
            std::cerr << "Unknown key: " << key << "\n";
        }        
    }
    return go;
}

Ray GeometryLoader::parseRayLine (const std::string& line) {
    GeometryObject go = parseLine(line);
    if (go.direction.magnitude() == 0) {
        std::cout << "Direction of ray was not initialized.\n";
        return Ray();
    }
    if (go.energyDensity == 0) {
        std::cerr << "Energy density of ray was not initialized.\n";
        return Ray();
    }
    Ray ray{go.origin, go.direction, go.energyDensity, go.refractiveIndex, go.wavelength};
    return ray;
}

std::vector<Ray> GeometryLoader::parseParallelRays (const std::string& line) {
    GeometryObject go = parseLine(line);
    if (go.direction.magnitude() * go.first.magnitude() * go.last.magnitude() == 0) { 
        throw std::invalid_argument("Invalid argument for parallel rays: " + line);; 
    }
    return makeParallelRays(go.direction, go.first, go.last, go.steps,
        go.energyDensity, 
        go.refractiveIndex, go.wavelength);
}

Lens GeometryLoader::parseSphericalLensLine(const std::string& line) {
    GeometryObject go = parseLine(line);
    std::unique_ptr<Material> m = go.createMaterial();
    
    if (go.r == 0) {
        throw std::invalid_argument("Zero radius in spherical lens: " + line);
    }

    return Lens::makeSphericalLens(Sphere(go.origin, go.r), std::move(m));
    
}

Lens GeometryLoader::parseConvexLensLine (const std::string& line) {
    GeometryObject go = parseLine(line);
    std::unique_ptr<Material> m = go.createMaterial();
    Lens l{Lens::makeConvexLens(go.origin, go.r, go.height, std::move(m))};
    l.setTransverseRadius(go.transverseRadius);
    return l;
}

Lens GeometryLoader::parsePlanoConvexLensLine (const std::string& line) {
    GeometryObject go = parseLine(line);
    std::unique_ptr<Material> m = go.createMaterial();
    Lens l{Lens::makePlanoConvexLens(go.origin, go.r, go.height, std::move(m))};
    l.setTransverseRadius(go.transverseRadius);
    return l;
}

Lens GeometryLoader::parsePlanoConcaveLensLine (const std::string& line) {
    GeometryObject go = parseLine(line);
    std::unique_ptr<Material> m = go.createMaterial();
    Lens l{Lens::makePlanoConcaveLens(go.origin, go.r, go.height, std::move(m))};
    l.setTransverseRadius(go.transverseRadius);
    return l;    
}

Lens GeometryLoader::parseConcaveLensLine (const std::string& line) {
    GeometryObject go = parseLine(line);
    std::unique_ptr<Material> m = go.createMaterial();
    Lens l{Lens::makeConcaveLens(go.origin, go.r, go.height, std::move(m))};
    l.setTransverseRadius(go.transverseRadius);
    return l;
}

Mirror GeometryLoader::parseMirror (const std::string& line) {
    GeometryObject go = parseLine(line);
    Mirror mirror{go.origin, go.a, go.b, go.reflectance};
    if (mirror.sideA.cross(mirror.sideB).magnitude() == 0) { 
        throw std::invalid_argument("Mirror sideA and sideB must be non-parallel: " + line);; 
    }
    mirror.surfaceNormal = mirror.sideA.cross(mirror.sideB).normalized();
    return mirror;
}

ParabolicMirror GeometryLoader::parseParabolicMirror (const std::string& line) {
    GeometryObject go = parseLine(line);
    return ParabolicMirror(go.origin, go.height, go.curvature, go.reflectance);
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