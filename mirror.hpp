#ifndef MIRROR_HPP
#define MIRROR_HPP

#include "mpvector.hpp"

class Mirror {
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
        Vector getOrigin();
        Vector getSideA();
        Vector getSideB();
        Vector getSurfaceNormal();
        double getReflectance();
};

#endif /* MIRROR_HPP */