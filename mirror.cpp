#include "mirror.hpp"
#include "mpvector.hpp"
#include "optdev.hpp"
#include "visualizeglfw.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

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

//////////

ParabolicMirror::ParabolicMirror() {
    ;
}

ParabolicMirror::ParabolicMirror(Vector origin_, Vector height_, double curvature_) {
    origin = origin_;
    height = height_;
    curvature = curvature_;
    // focalPoint = 
}
ParabolicMirror::ParabolicMirror(Vector origin_, Vector height_, Vector focalPoint_) {

}
Type ParabolicMirror::type() { return Type::Base; }

double ParabolicMirror::detectCollisionTime(const Ray& ray) const {
    // transform mirror and ray such that mirror is defined by k*(x^2+y^2) = z.
    // 1. Compute rotation matrix
    const Vector Z(0.0f, 0.0f, 1.0f);
    Vector axis = Z.cross(height.normalized());
    if (axis.magnitude() < 1e-6f) {
        axis = Vector(1.0f, 0.0f, 0.0f); // Handle parallel case
    }
    axis = axis.normalized();
    float rotationAngle = angle(Z, height);
    glm::mat3 R = glm::mat3(glm::rotate(glm::mat4(1.0f), rotationAngle, glm::vec3(axis)));

    // 2. Transform ray to parabola's local space
    Vector o_local = glm::transpose(R) * (ray.origin - origin);
    Vector v_local = glm::transpose(R) * ray.direction;

    std::cout << "oloc " << o_local.x << " " << o_local.y << " " << o_local.z << "\n";
    std::cout << "vloc " << v_local.x << " " << v_local.y << " " << v_local.z << "\n";

    // 3. Compute quadratic coefficients
    float A = curvature * (v_local.x * v_local.x + v_local.y * v_local.y);
    float B = 2.0f * curvature * (o_local.x * v_local.x + o_local.y * v_local.y) - v_local.z;
    float C = curvature * (o_local.x * o_local.x + o_local.y * o_local.y) - o_local.z;

    // 4. Solve quadratic
    float discriminant = B * B - 4 * A * C;
    std::cout << "DIS " << discriminant << "\n";
    if (discriminant < 0) return Inf; // No intersection

    float t = (-B - sqrt(discriminant)) / (2 * A);
    std::cout << "t " << t << "\n";

    if (t < Config::MIN_EPS) {
        t = (-B + sqrt(discriminant)) / (2 * A); // Try other solution
        std::cout << "t " << t << "\n";
        if (t < Config::MIN_EPS) return Inf; // Both solutions behind ray
        if (t > Config::MAX_T) return Inf;
    }
    return t;
}
std::vector<Ray> ParabolicMirror::createNewRays(const Ray& ray) const {
    std::vector<Ray> newRays;
    return newRays;
}
std::string ParabolicMirror::forPythonPlot() const {
    return "";
}
void ParabolicMirror::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {
    ;
}
