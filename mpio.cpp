#include <string>
#include <vector>
#include <sstream>
#include "ray.hpp"
#include "geometry_loader.hpp"

std::string printImports() {
    std::ostringstream oss;
    oss << "import numpy as np\n"
    << "import matplotlib.pyplot as plt\n"
    << "from matplotlib.patches import Circle\n"
    << "import matplotlib as mpl\n";
    return oss.str();
}

std::string printGeometry2D(GeometryLoader& geometry) {
    // return "A\n";
    std::ostringstream oss;
    oss << printImports();
    oss << "mpl.rcParams['lines.linewidth'] = 0.4\n"
    << "fig, ax = plt.subplots()\n";
    for (const auto& device : geometry.devices) {
        oss << device->forPythonPlot();
    }
    for (Ray& r: geometry.rays) {
        oss << r.forPythonPlot();
    }
    oss << "ax.set_aspect('equal', adjustable='box')\n"
    << "plt.show()\n";
    return oss.str();
}


std::string printRays(std::vector<Ray> rays) {
    std::ostringstream oss;
    oss << printImports();
    oss << "mpl.rcParams['lines.linewidth'] = 0.4\n"
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
