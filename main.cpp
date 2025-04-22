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
            rays[current].endT = MAX_T;
            rays[current].end = rays[current].getEndPoint();
            current++;
            continue;
        }

        OpticalDevice* collisionDevice = devices[min_index].get();
        rays[current].endT = *std::min_element(t_times.begin(), t_times.end());
        rays[current].end = rays[current].getEndPoint();

        std::vector<Ray> raysToAdd = collisionDevice->createNewRays(rays[current]);
        rays.insert(rays.end(),
        raysToAdd.begin(),
        raysToAdd.begin() + std::min(raysToAdd.size(), MAX_RAYS - rays.size()));

        current++;
    }
}

int main()
{
    std::vector<Ray> rays;
    rays.push_back(Ray(Vector(0,0,0.1), Vector(1,0,0), 1.));
    std::vector<Ray> raysToAdd;
    
    std::vector<std::unique_ptr<OpticalDevice>> devices;
    // devices.push_back(std::make_unique<Mirror>(Vector(5,-1,0), Vector(0,5,0), Vector(1,0,5), 1));
    devices.push_back(std::make_unique<SphericalLens>(Vector(10,0,0), 1.0, 1.33));
    devices.push_back(std::make_unique<SphericalLens>(Vector(12,0,0), 1.0, 1.33));

    rayTracing(rays, devices);

    // int current = 0;
    // while (current < rays.size()) {
    //     std::vector<double> t_times = rays[current].detectAllCollisionTimes(devices);

    //     auto it = std::min_element(std::begin(t_times), std::end(t_times));
    //     int min_index = std::distance(t_times.begin(), it);

    //     if (t_times[min_index] == Inf) {
    //         rays[current].endT = MAX_T;
    //         rays[current].end = rays[current].getEndPoint();
    //         current++;
    //         continue;
    //     }

    //     OpticalDevice* collisionDevice = devices[min_index].get();
    //     rays[current].endT = *std::min_element(t_times.begin(), t_times.end());
    //     rays[current].end = rays[current].getEndPoint();

    //     std::vector<Ray> raysToAdd = collisionDevice->createNewRays(rays[current]);
    //     rays.insert(rays.end(),
    //     raysToAdd.begin(),
    //     raysToAdd.begin() + std::min(raysToAdd.size(), MAX_RAYS - rays.size()));

    //     current++;
    // }

    std::cout << printRays(rays);
    
}