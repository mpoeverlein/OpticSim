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

class SurfaceGeometry {
    public:
        virtual ~SurfaceGeometry() = default;
        virtual double detectCollisionTime(const Ray& ray) const = 0;
        virtual Vector getSurfaceNormal(const Ray& ray) const = 0;
        virtual void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const = 0;
        virtual std::string toString() const = 0;
};

class Sphere {
    public:
        Vector origin;
        double radius;
        Sphere(Vector origin_, double radius_);
        Sphere(const SphereSection& ss);
};


class Disc : public SurfaceGeometry {
    public:
        Vector origin;
        Vector surfaceNormal;
        double radius;
        Disc();
        Disc(Vector origin_, Vector surfaceNormal_, double radius_);
        double detectCollisionTime(const Ray& ray) const;
        Vector getSurfaceNormal(const Ray& ray) const;
        void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
        std::string toString() const;
};

class Plane {
    public:
        Vector origin;
        Vector surfaceNormal;
        Plane(Vector origin_, Vector surfaceNormal_);
        Plane(const Disc& d);
};

class SphereSection : public SurfaceGeometry {
    public:
        Vector origin;
        double radius;
        Vector height; // direction of apex
        double openingAngle;
        SphereSection();
        SphereSection(Vector origin_, double radius_, Vector height_, double openingAngle_);
        SphereSection(const Sphere& s);
        double detectCollisionTime(const Ray& ray) const;
        Vector getSurfaceNormal(const Ray& ray) const;
        void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
        std::string toString() const;
};


// class Parallelogram : public SurfaceGeometry {
//     public:
//         Vector origin;
//         Vector sideA;
//         Vector sideB;
//         std::string toString() const;
// };

class Lens : public OpticalDevice {
    public:
        std::unique_ptr<Material> material;
        std::vector<std::unique_ptr<SurfaceGeometry>> surfaceGeometries;
        Lens();
        Lens(std::vector<std::unique_ptr<SurfaceGeometry>> surfaceGeometries_, std::unique_ptr<Material> material_);
        Lens(Sphere sphere1_, Sphere sphere2_, double refractiveIndex_);
        std::vector<double> determineCollisionTimes(const Ray& ray) const;
        double detectCollisionTime(const Ray& ray) const;
        std::string forPythonPlot() const;
        std::vector<Ray> createNewRays (const Ray& ray) const;
        void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
        static Lens makeSphericalLens(Sphere s, std::unique_ptr<Material> m);
        static Lens makeConvexLens(Vector origin_, double radius_, Vector height_, std::unique_ptr<Material> m);
        std::string toString() const;
};



// class PlanoConvex : public OpticalDevice {
//     public:
//         Vector origin = Vector(); // origin of sphere describing convex surface
//         double radius = 0; // radius of this sphere
//         double refractiveIndex = 1;
//         double reflectance = 0.1;
//         Vector height = Vector(); // height is the vector between the center of the planar surface and the apex of the convex surface
//         Vector planeOrigin;
//         // Vector sideA;
//         // Vector sideB;
//         double planeRadius;
//         Vector apex;
//         double openingAngle;
//         PlanoConvex(Vector planeOrigin_, double radius_, double n_, Vector height_);
//         void getBothCollisionTimes(const Ray& ray, double& t_plane, double& t_sphere) const;
//         double detectCollisionTime(const Ray& ray) const;
//         std::vector<Ray> createNewRays (const Ray& ray) const;
//         std::string forPythonPlot() const;
//         void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
// };



// class ConcaveLens : public OpticalDevice {
//     public:
//         Vector origin = Vector();
//         Vector sphere1Origin, sphere2Origin;
//         Vector apex1, apex2;
//         Vector height = Vector(1,0,0);
//         double openingAngle;
//         double radius = 0;
//         double refractiveIndex = 1;
//         double reflectance = 0.1;
//         ConcaveLens(Vector origin_, double radius_, double n_, Vector height_);
//         void getBothCollisionTimes(const Ray& ray, double& t1, double& t2) const;
//         double detectCollisionTime(const Ray& ray) const;
//         std::vector<Ray> createNewRays (const Ray& ray) const;
//         std::string forPythonPlot() const;
//         void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
// };

// class Aperture : public OpticalDevice {
//     public:
//         Vector origin = Vector();
//         Vector surfaceNormal = Vector(1,0,0);
//         double radius;
//         Aperture(Vector origin_, Vector surfaceNormal_, double radius_);
//         double detectCollisionTime(const Ray& ray) const;
//         std::vector<Ray> createNewRays (const Ray& ray) const;
//         std::string forPythonPlot() const;
//         void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
// };

std::vector<Ray> createNewRays (const Ray& ray, Vector surfaceNormal, double refractiveIndex, double reflectance);


#endif /* LENSES_HPP */