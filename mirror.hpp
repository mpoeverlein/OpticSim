#ifndef MIRROR_HPP
#define MIRROR_HPP

#include "mpvector.hpp"
#include "optdev.hpp"
#include "ray.hpp"

class OpticalDevice;

class Mirror : public OpticalDevice {
    public:
        Vector origin; // original start in meters
        Vector sideA; // 
        Vector sideB; //
        Vector surfaceNormal;
        double reflectance;
        double transmittance;
        // Mirror();
        Mirror(Vector origin_, Vector sideA_, Vector sideB_, double reflectance_);
        Type type();
        Vector getOrigin();
        Vector getSideA();
        Vector getSideB();
        Vector getSurfaceNormal();
        double getReflectance();
        double detectCollisionTime(const Ray& ray) const ;
};

#endif /* MIRROR_HPP */