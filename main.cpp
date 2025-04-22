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
        // for (const auto& dev: devices) {
        // for (OpticalDevice dev: devices) {
        for (const auto& device : devices) {
            switch (device->type()) {
            case Type::Mirror: {
                    Mirror* mirror = static_cast<Mirror*>(device.get());
                    std::cout << "MIRROR\n";
                    std::cout << mirror->getOrigin() << "\n";
                    t_times.push_back(rays[current].detectCollisionTime(*mirror));
                    break;
                }
                case Type::SphericalLens: {
                    SphericalLens* lens = static_cast<SphericalLens*>(device.get());
                    std::cout << "LENS\n";
                    std::cout << lens->getOrigin() << "\n";
                    t_times.push_back(rays[current].detectCollisionTime(*lens));
                    break;
                }
                case Type::Base:
                default:
                    break;
            }
        }

        for (double& t: t_times) {
            std::cout << t << "\n";
        }

        std::cout << "MIN" << *std::min_element(t_times.begin(), t_times.end()) << "\n";
        auto it = std::min_element(std::begin(t_times), std::end(t_times));
        std::cout << "MININDE" << std::distance(std::begin(t_times), it) << "\n";

        // for (Ray& r: raysToAdd) {
        //     if (rays.size() < MAX_RAYS) {
        //         rays.push_back(r);
        //     } else {
        //         break;
        //     }
        // }

        current++;
    }

    std::cout << printRays(rays);
    
}