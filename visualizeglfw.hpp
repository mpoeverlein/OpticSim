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


std::vector<Vertex> createSphere(
    const Vector& origin_,
    const Vector& upDirection,
    float radius,
    float openingAngle = M_PI,
    int segments = 16,
    const glm::vec3& color = glm::vec3(1.0f, 0.0f, 1.0f)
);
void visualizeWithGLFW(GeometryLoader& geometry);

#endif /* VISUALIZEWITHGLFW_HPP */