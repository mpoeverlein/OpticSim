#ifndef SURFACEGEOMETRY_HPP
#define SURFACEGEOMETRY_HPP

#include "mpvector.hpp"

class Ray; struct Vertex; class SphereSection; class Disc;

class Sphere {
    public:
        Vector origin;
        double radius;
        Sphere(Vector origin_, double radius_);
        Sphere(const SphereSection& ss);
};

class Plane {
    public:
        Vector origin;
        Vector surfaceNormal;
        Plane(Vector origin_, Vector surfaceNormal_);
        Plane(const Disc& d);
};

class SurfaceGeometry {
    public:
        virtual ~SurfaceGeometry() = default;
        virtual double detectCollisionTime(const Ray& ray) const = 0;
        virtual Vector getSurfaceNormal(const Ray& ray) const = 0;
        virtual void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const = 0;
        virtual std::string toString() const = 0;
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

class Parallelogram : public SurfaceGeometry {
    public:
        Vector origin;
        Vector sideA;
        Vector sideB;
        double detectCollisionTime(const Ray& ray) const;
        Vector getSurfaceNormal(const Ray& ray) const;
        void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
        std::string toString() const;
};

class Parabola : public SurfaceGeometry {
    public:
        Vector origin;
        Vector height;
        double curvature;
        double detectCollisionTime(const Ray& ray) const;
        Vector getSurfaceNormal(const Ray& ray) const;
        glm::mat3 getRotationMatrixForLocalCoordinates() const;
        void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
        std::string toString() const;
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

class CylinderSide : public SurfaceGeometry {
    public:
        Vector origin;
        Vector height;
        double radius;
        CylinderSide();
        CylinderSide(Vector origin_, Vector height_, double radius_);
        double detectCollisionTime(const Ray& ray) const;
        Vector getSurfaceNormal(const Ray& ray) const;
        void createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const;
        std::string toString() const;    
};


#endif /* SURFACEGEOMETRY_HPP */