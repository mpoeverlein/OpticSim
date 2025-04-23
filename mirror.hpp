#ifndef MIRROR_HPP
#define MIRROR_HPP

#include "mpvector.hpp"
#include "optdev.hpp"
#include "ray.hpp"
#include <vector>

class OpticalDevice;

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
};

#endif /* MIRROR_HPP */