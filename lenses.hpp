#ifndef LENSES_HPP
#define LENSES_HPP

#include "mpvector.hpp"
#include "optdev.hpp"
#include <iostream>

class SphericalLens : public OpticalDevice {
    private:
        Vector origin;
        double radius;
        double refractiveIndex;
    public:
        SphericalLens(Vector origin_, double radius_, double n_);
        Type type();
        friend std::ostream& operator<<(std::ostream& os, const SphericalLens& l);
        Vector getOrigin();
        double getRadius();
        double getRefractiveIndex();
};


#endif /* LENSES_HPP */