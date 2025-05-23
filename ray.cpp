#include "ray.hpp"
#include "mpvector.hpp"
#include <sstream>
#include <iomanip> 
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
    // std::string color;
    // if (refractiveIndex == 1) {
    //     color = "blue";
    // } else {
    //     color = "orange";
    // }
    glm::vec3 color = wavelengthToRGB(wavelength*1e9);
    std::cout << "# Wave " << wavelength*1e9 << "\n";
    oss << "ax.plot((" << origin.x << "," << end.x << "), (" 
        << origin.z << "," << end.z << "), linewidth=" 
        << std::setprecision(6) << 2*energyDensity << ", color=(" << color.x << ", " << color.y << ", " << color.z << ")"
        << ")\n"; // Adjust precision as needed
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

Vector Ray::getPositionAtTime(double t) const {
    return origin + t * direction;
}

bool operator==(Ray r1, Ray r2) {
    constexpr double epsilon = 1e-10;
    return r1.origin == r2.origin &&
           r1.direction == r2.direction &&
           std::abs(r1.energyDensity - r2.energyDensity) < epsilon &&
           std::abs(r1.wavelength - r2.wavelength) < epsilon &&
           std::abs(r1.refractiveIndex - r2.refractiveIndex) < epsilon;
}

bool operator!=(Ray r1, Ray r2) {
    return !(r1 == r2);
}