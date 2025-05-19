#include "material.hpp"
#include <sstream>

NonDispersiveMaterial::NonDispersiveMaterial(double n_) 
    : refractiveIndex(n_) {}

double NonDispersiveMaterial::getRefractiveIndex(double wavelength) const {
    return refractiveIndex;
}

std::ostream& operator<<(std::ostream& os, const NonDispersiveMaterial& m) {
    os << "Material: Non-dispersive, refractive index " << m.refractiveIndex << "\n";
    return os;
}