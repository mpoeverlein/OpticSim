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


int main()
{
    SphericalLens lens1(Vector(10,0,0), 1.0, 1.33); // refractive index of water at 20 deg C
    SphericalLens lens2(Vector(12,0,0), 1.0, 1.33); // refractive index of water at 20 deg C
    std::vector<SphericalLens> lenses = {lens1, lens2};

    Mirror m1(Vector(5,-1,0), Vector(0,5,0), Vector(0,0,5), 1);
    std::vector<Mirror> mirrors = {m1};

    std::vector<Ray> rays;
    rays.push_back(Ray(Vector(0,0,0.1), Vector(1,0,0), 1.));
    // std::vector<Ray> rays = makeParallelRays(Vector(1,0,0.0), Vector(0,0,-1), Vector(0,0,1), 100, 1.0, 1.0, 550e-9);
    std::vector<Ray> raysToAdd;
    // std::vector<std::unique_ptr<OpticalDevice>> devices;
    // devices.push_back(std::unique_ptr<OpticalDevice>(new Mirror(Vector(5,-1,0), Vector(0,5,0), Vector(0,0,5), 1)));

    // devices.push_back(std::unique_ptr<OpticalDevice>(new SphericalLens(Vector(10,0,0), 1.0, 1.33)));  // lens1
    // devices.push_back(std::unique_ptr<OpticalDevice>(new SphericalLens(Vector(12,0,0), 1.0, 1.33)));  // lens2
    std::vector<std::unique_ptr<OpticalDevice>> devices;
    devices.push_back(std::make_unique<Mirror>(Vector(5,-1,0), Vector(0,5,0), Vector(0,0,5), 1));
    devices.push_back(std::make_unique<SphericalLens>(Vector(10,0,0), 1.0, 1.33));
    devices.push_back(std::make_unique<SphericalLens>(Vector(12,0,0), 1.0, 1.33));

    int current = 0;
    while (current < rays.size()) {
        std::vector<double> t_times;
        for (const auto& device : devices) {
            t_times.push_back(device->detectCollisionTime(rays[current]));           
        }

        for (double& t: t_times) {
            std::cout << t << "\n";
        }

        std::cout << "MIN" << *std::min_element(t_times.begin(), t_times.end()) << "\n";
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

    std::cout << printRays(rays);
    
}