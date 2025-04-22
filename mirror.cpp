#include "mirror.hpp"
#include "mpvector.hpp"
#include "optdev.hpp"

// Mirror::Mirror () {
//     // all defaults already set
//     surfaceNormal = sideA.cross(sideB).normalized();
// }

Mirror::Mirror(Vector origin_, Vector sideA_, Vector sideB_, double reflectance_) {
    origin = origin_;
    sideA = sideA_;
    sideB = sideB_;
    surfaceNormal = sideA.cross(sideB).normalized();
    reflectance = reflectance_;
    transmittance = 1 - reflectance;
}

Type Mirror::type() { return Type::Mirror; }

Vector Mirror::getOrigin() { return origin; }
Vector Mirror::getSideA() { return sideA; }
Vector Mirror::getSideB() { return sideB; }
Vector Mirror::getSurfaceNormal() { return surfaceNormal; };
double Mirror::getReflectance() { return reflectance; }
