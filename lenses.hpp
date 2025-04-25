#ifndef LENSES_HPP
#define LENSES_HPP

#include "mpvector.hpp"
#include "optdev.hpp"
#include "ray.hpp"
#include <iostream>
#include <vector>

class OpticalDevice;
class Ray;
enum class Type;

class SphericalLens : public OpticalDevice {
    public:
        Vector origin = Vector();
        double radius = 0;
        double refractiveIndex = 1;
        SphericalLens();
        SphericalLens(Vector origin_, double radius_, double n_);
        Type type();
        friend std::ostream& operator<<(std::ostream& os, const SphericalLens& l);
        Vector getOrigin();
        double getRadius();
        double getRefractiveIndex();
        double detectCollisionTime(const Ray& ray) const;
        std::vector<Ray> createNewRays (const Ray& ray) const;
        std::string forPythonPlot() const;
};

class PlanoConvex : public OpticalDevice {
    public:
        Vector origin = Vector(); // origin of sphere describing convex surface
        double radius = 0; // radius of this sphere
        double refractiveIndex = 1;
        double reflectance = 0.5;
        Vector height = Vector(); // height is the vector between the center of the planar surface and the apex of the convex surface
        Vector planeOrigin;
        Vector sideA;
        Vector sideB;
        double planeRadius;
        Vector apex;
        double openingAngle;
        PlanoConvex(Vector origin_, double radius_, double n_, Vector height_);
        double detectCollisionTime(const Ray& ray) const;
        std::vector<Ray> createNewRays (const Ray& ray) const;
        std::string forPythonPlot() const;
};

std::vector<Ray> createNewRays (const Ray& ray, Vector surfaceNormal, double refractiveIndex, double reflectance);


#endif /* LENSES_HPP */