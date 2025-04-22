#ifndef optdev_HPP
#define optdev_HPP

#include <vector>

// #include "ray.hpp"
class Ray;

enum class Type {Base, Mirror, SphericalLens};


class OpticalDevice {
    public:
    virtual ~OpticalDevice() = default;
    virtual Type type();
    virtual double detectCollisionTime(const Ray& ray) const = 0;
    virtual std::vector<Ray> createNewRays(const Ray& ray) const = 0;
    virtual std::string forPythonPlot() const = 0;
};

#endif /* optdev_HPP */