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
    focalPoint = origin + 1 / (4*curvature) * height.normalized();
}

ParabolicMirror::ParabolicMirror(Vector origin_, Vector height_, Vector focalPoint_) {
    // origin = origin_;
    // height = height_;
    // focalPoint = focalPoint_;
    // curvature = 
}

ParabolicMirror::ParabolicMirror(Vector origin_, Vector height_, double curvature_, double reflectance_) {
    origin = origin_;
    height = height_;
    curvature = curvature_;
    reflectance = reflectance_;
    focalPoint = origin + 1 / (4*curvature) * height.normalized();
}

Type ParabolicMirror::type() { return Type::Base; }

glm::mat3 ParabolicMirror::getRotationMatrixForLocalCoordinates() const {
    const Vector Z(0.0f, 0.0f, 1.0f);
    Vector axis = Z.cross(height.normalized());
    if (axis.magnitude() < 1e-6f) {
        axis = Vector(1.0f, 0.0f, 0.0f); // Handle parallel case
    }
    axis = axis.normalized();
    float rotationAngle = angle(Z, height);
    return glm::mat3(glm::rotate(glm::mat4(1.0f), rotationAngle, glm::vec3(axis)));
}

double ParabolicMirror::detectCollisionTime(const Ray& ray) const {
    double t;
    // transform mirror and ray such that mirror is defined by k*(x^2+y^2) = z.
    // 1. Compute rotation matrix
    glm::mat3 R = getRotationMatrixForLocalCoordinates();

    // 2. Transform ray to parabola's local space
    Vector o_local = glm::transpose(R) * (ray.origin - origin);
    Vector d_local = glm::transpose(R) * ray.direction;

    // edge case: ray is perpendicular to mirror's height vector, z value in local coordinates is then given by x and y of o_local
    if (d_local.cross(Vector(0,0,1)).magnitude() < Config::MIN_EPS) {
        double z_hit = curvature*(o_local.x*o_local.x + o_local.y*o_local.y);
        if (z_hit > height.magnitude()) { return Inf; }
        t = (z_hit-o_local.z) / d_local.z;
        if (t < Config::MIN_EPS) {
            return Inf;
        }
        return t;
    }

    // 3. Compute quadratic coefficients
    double A = curvature * (d_local.x * d_local.x + d_local.y * d_local.y);
    double B = 2.0f * curvature * (o_local.x * d_local.x + o_local.y * d_local.y) - d_local.z;
    double C = curvature * (o_local.x * o_local.x + o_local.y * o_local.y) - o_local.z;

    // 4. Solve quadratic
    double discriminant = B * B - 4 * A * C;
    if (discriminant < 0) return Inf; // No intersection

    t = (-B - sqrt(discriminant)) / (2 * A);
    if ((o_local + t * d_local).z > height.magnitude()) { return Inf; }

    if (t < Config::MIN_EPS) {
        t = (-B + sqrt(discriminant)) / (2 * A); // Try other solution
        if ((o_local + t * d_local).z > height.magnitude()) { return Inf; }
        if (t < Config::MIN_EPS) return Inf; // Both solutions behind ray
        if (t > Config::MAX_T) return Inf;
    }
    return t;
}
std::vector<Ray> ParabolicMirror::createNewRays(const Ray& ray) const {
    std::vector<Ray> newRays;
    // surface normal in local coordinates given by gradient
    glm::mat3 R = getRotationMatrixForLocalCoordinates();
    Vector end_local = glm::transpose(R) * (ray.end - origin);
    Vector d_local = glm::transpose(R) * ray.direction;
    Vector surfaceNormal;

    surfaceNormal = -1 * Vector(2*curvature*end_local.x, 2*curvature*end_local.y, -1).normalized();
    Vector reflectionDirection = calculateReflectionDirection(d_local, surfaceNormal);
    reflectionDirection = R * reflectionDirection;
    newRays.push_back(Ray(ray.end, reflectionDirection, ray.energyDensity*reflectance, ray.refractiveIndex));
    return newRays;
}
std::string ParabolicMirror::forPythonPlot() const {
    return "";
}
void ParabolicMirror::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {
    ;
}
