#include "ray.hpp"
#include "mpvector.hpp"
#include <sstream>
#include <iomanip> 
#include "lenses.hpp"
#include "mirror.hpp"
#include "optdev.hpp"

Ray::Ray () {
    origin = Vector();
    direction = Vector();
    energyDensity = 0.0;
    refractiveIndex = 1.;
    wavelength = 550e-9;
}

Ray::Ray (Vector origin_, Vector direction_, double energyDensity_) {
    origin = origin_;
    direction = direction_.normalized();
    energyDensity = energyDensity_;
    refractiveIndex = 1.;
    wavelength = 550e-9;
}

Ray::Ray(Vector origin_, Vector direction_, double energyDensity_, double n) {
    origin = origin_;
    direction = direction_.normalized();
    energyDensity = energyDensity_;
    refractiveIndex = n;
    wavelength = 550e-9;    
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
    // oss << "ax.scatter(" << origin.x << "," << origin.z << ", marker='o')" << "\n";
    return oss.str();
}

std::vector<Ray> Ray::createReflectionAndRefraction (Vector surfaceNormal, Vector rotationAxis, double n2) {
    /* Create reflection and refraction rays according to Snell's law
    * n1 * sin(theta1) = n2 * sin(theta2)
    * surfaceNormal: direction of surface normal
    * rotationAxis: new rays created based on rotation of old about rotation axis
    * n2: refractive index of other medium (n1 is stored in the Ray object)
    */
    std::vector<Ray> newRays;
    double theta1 = angle(surfaceNormal, direction);
    double theta2 = refractiveIndex / n2 * sin(theta1);
    // we create a new direction for the ray by rotating
    // the old direction by theta2-theta1
    double dtheta = theta2 - theta1;
    Vector refractionDirection = rotateVectorAboutAxis(direction, rotationAxis, -dtheta);
    newRays.push_back(Ray(end, refractionDirection, energyDensity*0.98, n2));

    // create reflection
    Vector reflectionDirection = rotateVectorAboutAxis(direction, rotationAxis, -(M_PI-2*theta1));
    newRays.push_back(Ray(end, reflectionDirection, energyDensity*0.02, refractiveIndex));
    
    return newRays;
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

