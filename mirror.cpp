#include "mirror.hpp"
#include "mpvector.hpp"
#include "optdev.hpp"
#include "visualizeglfw.hpp"
#include <glm/glm.hpp>

#include <sstream>


Mirror::Mirror () {
    origin = Vector();
    sideA = Vector(1,0,0);
    sideB = Vector(0,1,0);
    surfaceNormal = sideA.cross(sideB).normalized();
    reflectance = 1;
    transmittance = 0;    
 }

Mirror::Mirror(Vector origin_, Vector sideA_, Vector sideB_, double reflectance_) {
    origin = origin_;
    sideA = sideA_;
    sideB = sideB_;
    surfaceNormal = sideA.cross(sideB).normalized();
    reflectance = reflectance_;
    transmittance = 1 - reflectance;
}

Type Mirror::type() { return Type::Mirror; }

double Mirror::detectCollisionTime(const Ray& ray) const {
    double t_hit = Inf;
    double alpha = Inf;
    double beta = Inf;
    calculateCollisionTime(ray.origin, ray.direction, origin, sideA, sideB, t_hit, alpha, beta);
    if (t_hit < Config::MIN_EPS) { return Inf; }
    if (t_hit == Inf) { return Inf; }

    if ( (alpha > 0) && (alpha < 1) && (beta > 0) && (beta < 1) ) {
        return t_hit;
    } else {
        return Inf;
    }
}

std::vector<Ray> Mirror::createNewRays (const Ray& ray) const {
    std::vector<Ray> newRays;
    Vector reflectionDirection = calculateReflectionDirection(ray.direction, surfaceNormal);
    // reflected ray
    newRays.push_back(Ray(ray.end, reflectionDirection, ray.energyDensity*reflectance, ray.refractiveIndex));
    // transmitted ray
    newRays.push_back(Ray(ray.end, ray.direction, ray.energyDensity*(1-reflectance), ray.refractiveIndex));
    return newRays;
}

std::string Mirror::forPythonPlot() const {
    std::ostringstream oss;
    oss << "ax.plot((" << origin.x << "," << origin.x+sideA.x << "), (" 
    << origin.z << "," << origin.z+sideA.z <<  "), linewidth=1, color='k')\n";
    oss << "ax.plot((" << origin.x << "," << origin.x+sideB.x << "), (" 
    << origin.z << "," << origin.z+sideB.z <<  "), linewidth=1, color='k')\n";
    return oss.str();
}

void Mirror::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {
    // vertices.push_back(Vertex{glm::vec3(origin.x,origin.y,origin.z), glm::vec3(surfaceNormal.x,surfaceNormal.y,surfaceNormal.z), glm::vec3(1.0,0,0), 0.4f});
    glm::vec3 color{1,0,0};
    float opacity = 0.4;
    vertices.push_back(Vertex{glm::vec3(origin), glm::vec3(surfaceNormal), color, opacity});
    vertices.push_back(Vertex{glm::vec3(origin+sideA), glm::vec3(surfaceNormal), color, opacity});
    vertices.push_back(Vertex{glm::vec3(origin+sideB), glm::vec3(surfaceNormal), color, opacity});
    vertices.push_back(Vertex{glm::vec3(origin+sideA+sideB), glm::vec3(surfaceNormal), color, opacity});
    unsigned int firstIndex = 0;
    if (indices.size() > 0) {
        firstIndex = *std::max_element(indices.begin(), indices.end()) + 1;
    }
    indices.insert(indices.end(), {firstIndex, firstIndex+1, firstIndex+2});
    indices.insert(indices.end(), {firstIndex+1, firstIndex+3, firstIndex+2});

}
