#ifndef LENSES_HPP
#define LENSES_HPP

#include "mpvector.hpp"
#include <iostream>

class SphericalLens {
    private:
        Vector origin;
        double radius;
        double refractiveIndex;
    public:
        SphericalLens(Vector origin_, double radius_, double n_);
        friend std::ostream& operator<<(std::ostream& os, const SphericalLens& l);
        Vector getOrigin();
        double getRadius();
        double getRefractiveIndex();
};


#endif /* LENSES_HPP */