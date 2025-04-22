#ifndef RAY_HPP
#define RAY_HPP

#include "mpvector.hpp"
#include "constants.hpp"
#include "lenses.hpp"
#include "mirror.hpp"
// #include "optdev.hpp"

class Ray {
    public:
        Vector origin = Vector(0,0,0); // original start in meters
        Vector direction = Vector(0,0,0); // vector travelled by ray in 1 second given in meters if travelling in vacuum
        // double wavelength;
        double energyDensity = 0; // energy density of photons in Joules per cubic meter
        double wavelength = 550e-9; // in m (describes color of photon)
        double refractiveIndex = 1.; //
        double startT = 0;
        double endT;
        Vector end = Vector(0,0,0);
        Ray();
        Ray(Vector origin_, Vector direction_, double energyDensity_);
        Ray(Vector origin_, Vector direction_, double energyDensity_, double n);       
        Ray(Vector origin_, Vector direction_, double energyDensity_, double n, double wavelength_);

        friend std::ostream& operator<<(std::ostream& os, const Ray& r) {
            os << "Ray: Origin: " << r.origin << " Direction: " << r.direction  
               << " t_End: " << r.endT << " " << " END: " << r.end
               << "\n";
            return os;
        }
        Vector getOrigin() { return origin; }
        Vector getDirection() { return direction; }
        double getRefractiveIndex() { return refractiveIndex; }
        Vector getEndPoint() { return origin + endT * direction; }
        std::string forPythonPlot();
        std::vector<Ray> createReflectionAndRefraction (Vector surfaceNormal, Vector rotationAxis, double n2);
        std::vector<double> detectAllCollisionTimes (const std::vector<std::unique_ptr<OpticalDevice>>& device) const;
};

std::vector<Ray> makeParallelRays(Vector direction, Vector first, Vector last, int steps,
    double energyDensity, double n, double wavelength);


#endif /* RAY_HPP */