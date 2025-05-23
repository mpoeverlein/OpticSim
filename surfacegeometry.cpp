#include "surfacegeometry.hpp"
#include <sstream>
#include <format>
#include <iomanip>
#include "visualizeglfw.hpp"
#include "ray.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

Disc::Disc() : origin(Vector()), surfaceNormal(Vector(1,0,0)), radius(1) {}

Disc::Disc(Vector origin_, Vector surfaceNormal_, double radius_) 
        : origin(origin_), surfaceNormal(surfaceNormal_), radius(radius_) {}

double Disc::detectCollisionTime(const Ray& ray) const {
    Plane tempPlane{origin, surfaceNormal};
    double t = calculateCollisionTime(ray, tempPlane);
    if ((ray.getPositionAtTime(t)-origin).magnitude() > radius) { t = Inf; }
    return t;
}

Vector Disc::getSurfaceNormal(const Ray& ray) const {
    return surfaceNormal;
}

void Disc::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {
    int segments = 16;
    std::vector<Vertex> discVerts = createDiscVertices(origin, surfaceNormal, radius);
    vertices.insert(vertices.end(), discVerts.begin(), discVerts.end());

    unsigned int current = 0; 
    if (indices.size() > 0) {
        current = *std::max_element(indices.begin(),indices.end())+1; 
    }
    std::vector<unsigned int> discIndices = createDiscIndices(segments, current);
    indices.insert(indices.end(), discIndices.begin(), discIndices .end());     
}

std::string Disc::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2); 
    
    oss << "Disc:\n"
        << "  Origin: " << origin.toString() << "\n"
        << "  Radius: " << radius << " m\n"
        << "  Normal vector: " << surfaceNormal.toString() << "\n";
    
    return oss.str();    
}

Plane::Plane(Vector origin_, Vector surfaceNormal_) : origin(origin_), surfaceNormal(surfaceNormal_) {}
Plane::Plane(const Disc& d) : origin(d.origin), surfaceNormal(d.surfaceNormal) {}

glm::mat3 Parabola::getRotationMatrixForLocalCoordinates() const {
    const Vector Z(0.0f, 0.0f, 1.0f);
    Vector axis = height.normalized().cross(Z);
    if (axis.magnitude() < 1e-6f) {
        axis = Vector(1.0f, 0.0f, 0.0f); // Handle parallel case
    }
    axis = axis.normalized();
    float rotationAngle = angle(Z, height);
    // std::cout << "# AXIS " << axis << " angle " << rotationAngle*180/M_PI << "\n";
    return glm::mat3(glm::rotate(glm::mat4(1.0f), rotationAngle, glm::vec3(axis)));    
}

double Parabola::detectCollisionTime(const Ray& ray) const {
    return 0;
}
Vector Parabola::getSurfaceNormal(const Ray& ray) const {
    return Vector();
}
void Parabola::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {
    ;
}
std::string Parabola::toString() const {
    return "Parabola\n";
}


Sphere::Sphere(Vector origin_, double radius_)
    : origin(origin_),
      radius(radius_)
{
    if (radius_ == 0) {
        std::cerr << "Sphere radius cannot be zero. Entered value: " << radius_ << "\n";
    }    
}

Sphere::Sphere(const SphereSection& ss) 
        : origin(ss.origin), radius(ss.radius) {}

SphereSection::SphereSection() 
    : origin(Vector()),
      radius(1),
      height(Vector(0,0,1)),  // Normalize height vector
      openingAngle(M_PI)
{

}

SphereSection::SphereSection(Vector origin_, double radius_, Vector height_, double openingAngle_)
    : origin(origin_),
      radius(radius_),
      height(height_.normalized()*radius),  // Normalize height vector
      openingAngle(openingAngle_)
{
    // Validate parameters
    if (radius_ == 0) {
        std::cerr << "SphereSection radius cannot be zero. Entered value: " << radius_ << "\n";
    }
    if (openingAngle_ <= 0 || openingAngle_ > M_PI) {
        std::cerr << "SphereSection Opening angle must be between 0 and π radians. Entered value: " << openingAngle_ << "\n";
    }
    if (height_.magnitude() == 0) {
        std::cerr << "SphereSection Height vector cannot be zero. Entered value: " << height_ << "\n";
    }
}

SphereSection::SphereSection(const Sphere& s)
        : origin(s.origin), radius(s.radius), height(Vector(0,0,s.radius)), openingAngle(M_PI) {}

double SphereSection::detectCollisionTime(const Ray& ray) const {
    return calculateCollisionTime(ray, *this);
}

Vector SphereSection::getSurfaceNormal(const Ray& ray) const {
    return (origin - ray.end).normalized();
}

void SphereSection::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {
    // std::cout << "creating vertices" << radius << " opening angle " << openingAngle << "\n";
    // std::cout << " for " << origin << " " << height.normalized() << "\n";
    int segments = 16;
    std::vector<Vertex> sphereVerts = createSphereVertices(origin, height.normalized(), abs(radius), openingAngle, segments);
    vertices.insert(vertices.end(), sphereVerts.begin(), sphereVerts.end());

    unsigned int current = 0; 
    if (indices.size() > 0) {
        current = *std::max_element(indices.begin(),indices.end())+1; 
    }
    std::vector<unsigned int> sphereIndices = createSphereIndices(segments, current);
    indices.insert(indices.end(), sphereIndices.begin(), sphereIndices.end());    
}



std::string SphereSection::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2); // Control decimal places
    
    oss << "Sphere Section:\n"
        << "  Origin: " << origin.toString() << "\n"
        << "  Radius: " << radius << " m\n"
        << "  Height: " << height.toString() << "\n"
        << "  Angle: " << (openingAngle/M_PI*180) << "°\n";
    std::cout << oss.str();
    return oss.str();
}
