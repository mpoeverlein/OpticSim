#include <stdio.h>
#include <vector>
#include <iomanip> 
#include "mpvector.hpp"
#include "ray.hpp"
#include "constants.hpp"
#include "lenses.hpp"
#include "mirror.hpp"
#include "optdev.hpp"
#include "mpio.hpp"
#include "config_loader.hpp"
#include "geometry_loader.hpp"
#include <typeinfo>
#include <memory>  // For std::shared_ptr or std::unique_ptr

void rayTracing(std::vector<Ray>& rays, const std::vector<std::unique_ptr<OpticalDevice>>& devices) {
    std::vector<Ray> raysToAdd;
    int current = 0;
    while (current < rays.size()) {
        std::vector<double> t_times = rays[current].detectAllCollisionTimes(devices);

        auto it = std::min_element(std::begin(t_times), std::end(t_times));
        int min_index = std::distance(t_times.begin(), it);

        if (t_times[min_index] == Inf) {
            rays[current].endT = Config::MAX_T;
            rays[current].end = rays[current].getEndPoint();
            current++;
            continue;
        }

        OpticalDevice* collisionDevice = devices[min_index].get();
        rays[current].endT = *std::min_element(t_times.begin(), t_times.end());
        rays[current].end = rays[current].getEndPoint();

        std::vector<Ray> raysToAdd = collisionDevice->createNewRays(rays[current]);

        for (Ray& ray: raysToAdd) {
            if (ray.energyDensity < Config::MIN_ENERGY_DENSITY) { continue; }
            rays.push_back(ray);
        }

        current++;
    }
}

int main()
{
    ConfigLoader::loadFromFile("config.conf");
    // GeometryLoader geometry;
    // geometry.loadFromFile("geometry.geo");

    // rayTracing(geometry.rays, geometry.devices);
    // std::cout << printGeometry2D(geometry);

    std::vector<Ray> rays = makeParallelRays(Vector(1,0,0), Vector(0,0,-2), Vector(0,0,2), 100,
        1, 1., 550e-9);
    // std::vector<Ray> rays = {Ray(Vector(3.2,0,0), Vector(1,0,0), 1, 1, 550e-9)};
    std::vector<std::unique_ptr<OpticalDevice>> devices;
    // // devices.push_back(std::make_unique<PlanoConvex>(Vector(5,0,0), 1, 1.5, Vector(-0.9,0,0)));
    // devices.push_back(std::make_unique<ConvexLens>(Vector(5,0,0), 1, 1.5, Vector(-0.2,0,0)));
    // devices.push_back(std::make_unique<ConcaveLens>(Vector(5,0,0), 1, 1.5, Vector(-0.2,0,0)));
    devices.push_back(std::make_unique<Aperture>(Vector(2,0,0), Vector(1,0,0), 0.2));
    rayTracing(rays, devices);
    std::cout << printRays(rays);
    // std::cout << devices[0]->forPythonPlot();
}