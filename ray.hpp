#ifndef RAY_HPP
#define RAY_HPP

#include "mpvector.hpp"
#include "constants.hpp"

class OpticalDevice;

class Ray {
    public:
        Vector origin = Vector(0,0,0); // original start in meters
        Vector direction = Vector(0,0,0); // vector travelled by ray in 1 second given in meters if travelling in vacuum
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
        Vector getEndPoint() { return origin + endT * direction; }
        std::string forPythonPlot();
        std::vector<double> detectAllCollisionTimes (const std::vector<std::unique_ptr<OpticalDevice>>& device) const;
        Vector getPositionAtTime(double t) const ;
};

std::vector<Ray> makeParallelRays(Vector direction, Vector first, Vector last, int steps,
    double energyDensity, double n, double wavelength);
bool operator==(Ray r1, Ray r2);
bool operator!=(Ray r1, Ray r2);


#endif /* RAY_HPP */