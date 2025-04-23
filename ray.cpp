#include "ray.hpp"
#include "mpvector.hpp"
#include <sstream>
#include <iomanip> 
#include "lenses.hpp"
#include "mirror.hpp"
#include "optdev.hpp"

Ray::Ray () {
    // use default values of ray.hpp
 }

Ray::Ray (Vector origin_, Vector direction_, double energyDensity_) {
    origin = origin_;
    direction = direction_.normalized();
    energyDensity = energyDensity_;
}

Ray::Ray(Vector origin_, Vector direction_, double energyDensity_, double n) {
    origin = origin_;
    direction = direction_.normalized();
    energyDensity = energyDensity_;
    refractiveIndex = n;
}

Ray::Ray(Vector origin_, Vector direction_, double energyDensity_, double n, double wavelength_) {
    origin = origin_;
    direction = direction_.normalized();
    energyDensity = energyDensity_;
    refractiveIndex = n;
    wavelength = wavelength_;     
}

std::string Ray::forPythonPlot () {
    std::ostringstream oss;
    std::string color;
    if (refractiveIndex == 1) {
        color = "blue";
    } else {
        color = "orange";
    }
    oss << "ax.plot((" << origin.x << "," << end.x << "), (" 
        << origin.z << "," << end.z << "), linewidth=" 
        << std::setprecision(6) << 2*energyDensity << ", color='" << color << "')\n"; // Adjust precision as needed
    return oss.str();
}

std::vector<double> Ray::detectAllCollisionTimes(const std::vector<std::unique_ptr<OpticalDevice>>& devices) const {
    std::vector<double> t_times;
    for (const auto& device : devices) {
        t_times.push_back(device->detectCollisionTime(*this));           
    }
    return t_times;
}

std::vector<Ray> makeParallelRays(Vector direction, Vector first, Vector last, int steps,
    double energyDensity, double n, double wavelength) {
    std::vector<Ray> rays;
    float ratio;
    Vector final = last - first;
    for (int i = 0; i < steps; i++) {
        ratio = (float) i / steps;
        rays.push_back(Ray(first+ratio*final, direction, energyDensity, n, wavelength));
    }
    return rays;
}

