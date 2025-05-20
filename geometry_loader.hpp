#ifndef GEOMETRY_LOADER_HPP
#define GEOMETRY_LOADER_HPP

#include "mpvector.hpp"

class Ray;
class OpticalDevice;
class SphericalLens;
class Mirror;
class ParabolicMirror;
class ConvexLens;

#include <string>
#include <unordered_map>

class GeometryObject {
    public:
        Vector origin;
        Vector height;
        Vector direction;
        double energyDensity = 1;
        double refractiveIndex = 1;
        double wavelength = 550e-9;
        Vector first;
        Vector last;
        double steps = 1;
        double r = 1;
        Vector a;
        Vector b;
        double curvature = 1;
        double reflectance = 1;
        std::string material = "none";
        double temperature = 298.15;
};

class GeometryLoader {
public:
    std::vector<Ray> rays;
    std::vector<std::unique_ptr<OpticalDevice>> devices;
    void loadFromFile (const std::string& filename);
    static Vector parseVector (const std::string& str);
    static GeometryObject parseLine (const std::string& str);
    static Ray parseRayLine (const std::string& line);
    static std::vector<Ray> parseParallelRays (const std::string& line);
    static SphericalLens parseSphericalLensLine (const std::string& line);
    static ConvexLens parseConvexLensLine (const std::string& line);
    static Mirror parseMirror (const std::string& line); 
    static ParabolicMirror parseParabolicMirror (const std::string& line); 
};

#endif /* GEOMETRY_LOADER_HPP */