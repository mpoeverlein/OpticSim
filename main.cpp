#include <stdio.h>
#include <vector>
#include <sstream>
#include <iomanip> 
#include "mpvector.hpp"
#include "ray.hpp"
#include "constants.hpp"
#include "lenses.hpp"
#include "mirror.hpp"

std::string printRays(std::vector<Ray> rays) {
    std::ostringstream oss;
    oss << "import numpy as np\n"
    << "import matplotlib.pyplot as plt\n"
    << "from matplotlib.patches import Circle\n"
    << "import matplotlib as mpl\n"
    << "mpl.rcParams['lines.linewidth'] = 0.4\n"
    << "fig, ax = plt.subplots()\n"
    << "circ = Circle((10, 0.), 1, alpha=0.05, ec='blue')\n"
    << "ax.add_patch(circ)\n";

    for (Ray& r: rays) {
        oss << r.forPythonPlot();
    }
    oss << "ax.set_xlim((8.9,11.1))\n"
        << "ax.set_ylim((-1.1,1.1))\n"
        << "ax.set_aspect('equal', adjustable='box')\n";
    oss << "plt.show()\n";
        
    return oss.str();
}


int main()
{
    // Ray ray(Vector(0,0,0), Vector(1,0,0.05), 0.1, 1.0, 550e-9);
    SphericalLens lens1(Vector(10,0,0), 1.0, 1.33); // refractive index of water at 20 deg C
    SphericalLens lens2(Vector(12,0,0), 1.0, 1.33); // refractive index of water at 20 deg C
    std::vector<SphericalLens> lenses = {lens1, lens2};

    Mirror m1(Vector(5,-1,0), Vector(0,5,0), Vector(0,0,5), 1);
    // Mirror m1(Vector(5,-2,-2), Vector(0,5,0), Vector(5,0,0), 1);
    std::vector<Mirror> mirrors = {m1};

    std::vector<Ray> rays = makeParallelRays(Vector(1,0,0.0), Vector(0,0,-1), Vector(0,0,1), 100, 1.0, 1.0, 550e-9);
    std::vector<Ray> raysToAdd;
    int current = 0;
    while (current < rays.size()) {

        // for (Mirror& m: mirrors) {
        //     raysToAdd = rays[current].createRayFromNewCollision(m);
        //     if (raysToAdd.size() > 0) {
        //         break;
        //     }
        // }
        
        // if (raysToAdd.size() == 0) {
        //     for (SphericalLens& lens: lenses) {
        //         raysToAdd = rays[current].createRayFromNewCollision(lens);
        //         if (raysToAdd.size() > 0) {
        //             break;
        //         }
        //     }
        // }

        for (SphericalLens& lens: lenses) {
            raysToAdd = rays[current].createRayFromNewCollision(lens);
            if (raysToAdd.size() > 0) {
                break;
            }
        }


        for (Ray& r: raysToAdd) {
            if (rays.size() < MAX_RAYS) {
                rays.push_back(r);
            } else {
                break;
            }
        }

        current++;
    }

    std::cout << printRays(rays);
    
}