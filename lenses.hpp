#ifndef LENSES_HPP
#define LENSES_HPP

#include "mpvector.hpp"
#include "optdev.hpp"
#include "ray.hpp"
#include <iostream>
#include <vector>

class OpticalDevice;
class Ray;
enum class Type;

class SphericalLens : public OpticalDevice {
    public:
        Vector origin = Vector();
        double radius = 0;
        double refractiveIndex = 1;
        SphericalLens();
        SphericalLens(Vector origin_, double radius_, double n_);
        Type type();
        friend std::ostream& operator<<(std::ostream& os, const SphericalLens& l);
        Vector getOrigin();
        double getRadius();
        double getRefractiveIndex();
        double detectCollisionTime(const Ray& ray) const;
        std::vector<Ray> createNewRays (const Ray& ray) const;
};


#endif /* LENSES_HPP */