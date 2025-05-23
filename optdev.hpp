#ifndef optdev_HPP
#define optdev_HPP

#include <vector>
#include "surfacegeometry.hpp"
#include "material.hpp"
#include "mpvector.hpp"

class Ray;
struct Vertex;


class OpticalDevice {
    public:
    virtual ~OpticalDevice() = default;
    virtual double detectCollisionTime(const Ray& ray) const = 0;
    virtual std::vector<Ray> createNewRays(const Ray& ray) const = 0;
    virtual std::string forPythonPlot() const = 0;
    virtual void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const = 0;
    virtual std::string toString() const = 0;
};

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

class Mirror : public OpticalDevice {
    public:
        Vector origin; // original start in meters
        Vector sideA; // 
        Vector sideB; //
        Vector surfaceNormal;
        double reflectance = 1;
        double transmittance = 0;
        Mirror();
        Mirror(Vector origin_, Vector sideA_, Vector sideB_, double reflectance_);
        double detectCollisionTime(const Ray& ray) const ;
        std::vector<Ray> createNewRays(const Ray& ray) const;
        std::string forPythonPlot() const;
        void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
        std::string toString() const;
};

class ParabolicMirror : public OpticalDevice {
    public:
        Vector origin;
        Vector height;
        Vector focalPoint;
        double curvature;
        double reflectance = 1;
        double transmittance = 0;
        ParabolicMirror();
        ParabolicMirror(Vector origin_, Vector height_, double curvature_);
        ParabolicMirror(Vector origin_, Vector height_, Vector focalPoint_);
        ParabolicMirror(Vector origin_, Vector height_, double curvature_, double reflectance_);
        glm::mat3 getRotationMatrixForLocalCoordinates() const ;
        double detectCollisionTime(const Ray& ray) const ;
        std::vector<Ray> createNewRays(const Ray& ray) const;
        std::string forPythonPlot() const;
        void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
        std::string toString() const;
};

std::vector<Ray> createNewRays (const Ray& ray, Vector surfaceNormal, double refractiveIndex, double reflectance);

#endif /* optdev_HPP */