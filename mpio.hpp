#ifndef MPIO_HPP
#define MPIO_HPP

#include "ray.hpp"
#include "geometry_loader.hpp"

std::string printRays(std::vector<Ray> rays);
std::string printGeometry2D(GeometryLoader& geometry);

#endif /* MPIO_HPP */