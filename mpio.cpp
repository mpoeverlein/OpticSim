#include <string>
#include <vector>
#include <sstream>
#include "ray.hpp"
#include "geometry_loader.hpp"
#include "optdev.hpp"

/**
 * @return Python import statements as a string
 */
std::string printImports() {
    std::ostringstream oss;
    oss << "import numpy as np\n"
    << "import matplotlib.pyplot as plt\n"
    << "import matplotlib.patches as patches\n"
    << "import matplotlib as mpl\n";
    return oss.str();
}

/**
 * @return string for Python script to plot rays and optical devices.
 */
std::string printGeometry2D(GeometryLoader& geometry) {
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

/**
 * @return string for Python script of plotting rays.
 */
std::string printRays(std::vector<Ray> rays) {
    std::ostringstream oss;
    oss << printImports();
    oss << "mpl.rcParams['lines.linewidth'] = 0.4\n"
    << "fig, ax = plt.subplots()\n";

    for (Ray& r: rays) {
        oss << r.forPythonPlot();
    }
    oss << "# ax.set_xlim((8.9,11.1))\n"
        << "# ax.set_ylim((-1.1,1.1))\n"
        << "ax.set_aspect('equal', adjustable='box')\n";
    oss << "plt.show()\n";
        
    return oss.str();
}
