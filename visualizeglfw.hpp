#ifndef VISUALIZEWITHGLFW_HPP
#define VISUALIZEWITHGLFW_HPP


#include <glm/glm.hpp>
#include <vector>

class GeometryLoader;

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 color;
    float alpha;
};

class Vector;

std::vector<Vertex> createCylinderSideVertices(
    const Vector& start_, 
    const Vector& end_, 
    float radius, 
    int segments = 16,
    const glm::vec3& color = glm::vec3(1.0f, 1.0f, 1.0f),
    float opacity = 1.0f
);

std::vector<unsigned int> createCylinderSideIndices(int segments, unsigned int base);

std::vector<Vertex> createSphereVertices(
    const Vector& origin_,
    const Vector& upDirection,
    float radius,
    float openingAngle = M_PI,
    int segments = 16,
    const glm::vec3& color = glm::vec3(1.0f, 0.0f, 1.0f),
    float opacity = 1.0f
);

std::vector<unsigned int> createSphereIndices(int segments, unsigned int base);

std::vector<Vertex> createParabolaVertices(
    const Vector& origin_,
    const Vector& height,
    float curvature,
    int segments,
    const glm::vec3& color,
    float opacity = 1.0f
);

std::vector<Vertex> createDiscVertices(
    const Vector& origin_,
    const Vector& surfaceNormal_,
    const double radius_,
    int segments = 16,
    const glm::vec3& color = glm::vec3(1.0f, 0.0f, 1.0f),
    float opacity = 1.0f
);

std::vector<unsigned int> createDiscIndices(int segments, unsigned int base);

void visualizeWithGLFW(GeometryLoader& geometry);

#endif /* VISUALIZEWITHGLFW_HPP */