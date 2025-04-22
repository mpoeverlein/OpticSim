#ifndef MIRROR_HPP
#define MIRROR_HPP

#include "mpvector.hpp"
#include "optdev.hpp"


class Mirror : public OpticalDevice {
    private:
        Vector origin; // original start in meters
        Vector sideA; // 
        Vector sideB; //
        Vector surfaceNormal;
        double reflectance;
        double transmittance;
    public:
        // Mirror();
        Mirror(Vector origin_, Vector sideA_, Vector sideB_, double reflectance_);
        Type type();
        Vector getOrigin();
        Vector getSideA();
        Vector getSideB();
        Vector getSurfaceNormal();
        double getReflectance();
};

#endif /* MIRROR_HPP */