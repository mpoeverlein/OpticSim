#ifndef GEOMETRY_LOADER_HPP
#define GEOMETRY_LOADER_HPP

class Ray;
class OpticalDevice;
class Vector;
class SphericalLens;
class Mirror;

#include <string>
#include <unordered_map>

class GeometryLoader {
public:
    std::vector<Ray> rays;
    std::vector<std::unique_ptr<OpticalDevice>> devices;
    void loadFromFile(const std::string& filename);
    static Vector parseVector(const std::string& str);
    static Ray parseRayLine (const std::string& line);
    static std::vector<Ray> parseParallelRays (const std::string& line);
    static SphericalLens parseSphericalLensLine (const std::string& line);
    static Mirror parseMirror (const std::string& line); 
};

#endif /* GEOMETRY_LOADER_HPP */