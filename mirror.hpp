#ifndef MIRROR_HPP
#define MIRROR_HPP

#include "mpvector.hpp"
#include "optdev.hpp"
#include "ray.hpp"
#include <vector>

class OpticalDevice;
struct Vertex;

class Mirror : public OpticalDevice {
    public:
        Vector origin; // original start in meters
        Vector sideA; // 
        Vector sideB; //
        Vector surfaceNormal;
        double reflectance = 1;
        double transmittance = 0;
        Mirror();
        Mirror(Vector origin_, Vector sideA_, Vector sideB_, double reflectance_);
        Type type();
        double detectCollisionTime(const Ray& ray) const ;
        std::vector<Ray> createNewRays(const Ray& ray) const;
        std::string forPythonPlot() const;
        void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
};

#endif /* MIRROR_HPP */