#ifndef LENSES_HPP
#define LENSES_HPP

#include "mpvector.hpp"
#include "optdev.hpp"
#include "ray.hpp"
#include "material.hpp"
#include <iostream>
#include <vector>
#include <functional>

class OpticalDevice;
class Ray;

class Sphere {
    public:
        Vector origin;
        double radius;
        Sphere(Vector origin_, double radius_);
};

class SphereSection {
    public:
        Vector origin;
        double radius;
        Vector height; // direction of apex
        double openingAngle;
        SphereSection();
        SphereSection(Vector origin_, double radius_, Vector height_, double openingAngle_);
};

class Lens : public OpticalDevice {
    public:
        std::unique_ptr<Material> material;
        SphereSection sphereSection1;
        SphereSection sphereSection2;
        Lens(Sphere sphere1_, Sphere sphere2_, double refractiveIndex_);
        // Lens(Sphere sphere1_, Sphere sphere2_, std::unique_ptr<Material> material);
        double detectCollisionTime(const Ray& ray) const;
        std::string forPythonPlot() const;
        std::vector<Ray> createNewRays (const Ray& ray) const;
        void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
};

class SphericalLens : public OpticalDevice {
    public:
        Vector origin = Vector();
        double radius = 0;
        std::unique_ptr<Material> material; 

        SphericalLens();
        SphericalLens(Vector origin_, double radius_, double n_);
        SphericalLens(Vector origin_, double radius_, std::unique_ptr<Material> material);
        friend std::ostream& operator<<(std::ostream& os, const SphericalLens& l);
        Vector getOrigin();
        double getRadius();
        double getRefractiveIndex(double wavelength);
        double detectCollisionTime(const Ray& ray) const;
        std::vector<Ray> createNewRays (const Ray& ray) const;
        std::vector<Ray> createNewRaysInsideOut (const Ray& ray) const;
        std::string forPythonPlot() const;
        void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
};

class PlanoConvex : public OpticalDevice {
    public:
        Vector origin = Vector(); // origin of sphere describing convex surface
        double radius = 0; // radius of this sphere
        double refractiveIndex = 1;
        double reflectance = 0.1;
        Vector height = Vector(); // height is the vector between the center of the planar surface and the apex of the convex surface
        Vector planeOrigin;
        // Vector sideA;
        // Vector sideB;
        double planeRadius;
        Vector apex;
        double openingAngle;
        PlanoConvex(Vector planeOrigin_, double radius_, double n_, Vector height_);
        void getBothCollisionTimes(const Ray& ray, double& t_plane, double& t_sphere) const;
        double detectCollisionTime(const Ray& ray) const;
        std::vector<Ray> createNewRays (const Ray& ray) const;
        std::string forPythonPlot() const;
        void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
};

/** The convex lens is defined by the intersection of two spheres of the same size */
class ConvexLens : public OpticalDevice {
    public:
        Vector origin = Vector();
        Vector sphere1Origin, sphere2Origin;
        Vector apex1, apex2;
        Vector height = Vector(1,0,0);
        double radius = 0; // radius of both spheres
        double refractiveIndex = 1;
        double reflectance = 0.1;
        double openingAngle;
        ConvexLens();
        ConvexLens(Vector origin_, double radius_, double n_, Vector height_);
        void getBothCollisionTimes(const Ray& ray, double& t1, double& t2) const;
        double detectCollisionTime(const Ray& ray) const;
        std::vector<Ray> createNewRays (const Ray& ray) const;
        std::string forPythonPlot() const;       
        void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const; 
};

class ConcaveLens : public OpticalDevice {
    public:
        Vector origin = Vector();
        Vector sphere1Origin, sphere2Origin;
        Vector apex1, apex2;
        Vector height = Vector(1,0,0);
        double openingAngle;
        double radius = 0;
        double refractiveIndex = 1;
        double reflectance = 0.1;
        ConcaveLens(Vector origin_, double radius_, double n_, Vector height_);
        void getBothCollisionTimes(const Ray& ray, double& t1, double& t2) const;
        double detectCollisionTime(const Ray& ray) const;
        std::vector<Ray> createNewRays (const Ray& ray) const;
        std::string forPythonPlot() const;
        void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
};

class Aperture : public OpticalDevice {
    public:
        Vector origin = Vector();
        Vector surfaceNormal = Vector(1,0,0);
        double radius;
        Aperture(Vector origin_, Vector surfaceNormal_, double radius_);
        double detectCollisionTime(const Ray& ray) const;
        std::vector<Ray> createNewRays (const Ray& ray) const;
        std::string forPythonPlot() const;
        void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
};

std::vector<Ray> createNewRays (const Ray& ray, Vector surfaceNormal, double refractiveIndex, double reflectance);


#endif /* LENSES_HPP */