#include "optdev.hpp"
#include "lenses.hpp"
#include "mpvector.hpp"
#include "ray.hpp"
#include <sstream>
#include <iomanip> 

SphericalLens::SphericalLens() {}

SphericalLens::SphericalLens(Vector origin_, double radius_, double n_) {
    origin = origin_;
    radius = radius_;
    refractiveIndex = n_;
}

Type SphericalLens::type() { return Type::SphericalLens; }
Vector SphericalLens::getOrigin() { return origin; }
double SphericalLens::getRadius() { return radius; }
double SphericalLens::getRefractiveIndex() { return refractiveIndex; }

double SphericalLens::detectCollisionTime(const Ray& ray) const {
   if (ray.energyDensity < Config::MIN_ENERGY_DENSITY) { return Inf; }
   return calculateCollisionTime(ray.origin, ray.direction, origin, radius);
}

std::vector<Ray> SphericalLens::createNewRays (const Ray& ray) const {
    std::vector<Ray> newRays;
    Vector surfaceNormal;
    double n2;
    if (ray.refractiveIndex != refractiveIndex) {
        // outside
        surfaceNormal = (origin - ray.end).normalized();
        n2 = refractiveIndex;
    } else {
        // inside
        surfaceNormal = (ray.end - origin).normalized();
        n2 = 1.;
    }

    return ::createNewRays(ray, surfaceNormal, n2, 0); // reflectance set to 0
}

std::vector<Ray> SphericalLens::createNewRaysInsideOut (const Ray& ray) const {
    std::vector<Ray> newRays;
    Vector surfaceNormal;
    double n2;
    if (ray.refractiveIndex != refractiveIndex) {
        // outside
        surfaceNormal = (ray.end - origin).normalized();
        n2 = refractiveIndex;
    } else {
        // inside
        surfaceNormal = (origin - ray.end).normalized();
        n2 = Config::VACUUM_REFRACTIVE_INDEX;
    }

    return ::createNewRays(ray, surfaceNormal, n2, 0); // reflectance set to 0
}


std::string SphericalLens::forPythonPlot() const {
    std::ostringstream oss;
    oss << "circ = patches.Circle((" << origin.x << ", " << origin.z << "), " << radius << ", alpha=0.05, ec='blue')\n"
    << "ax.add_patch(circ)\n";
    return oss.str();
}


////////////////////////////////


    
PlanoConvex::PlanoConvex(Vector planeOrigin_, double radius_, double n_, Vector height_) {
    planeOrigin = planeOrigin_;
    radius = radius_;
    refractiveIndex = n_;
    height = height_;
    if (height.magnitude() > radius) {
        throw "Height vector must not be longer than sphere radius!";
    }
    origin = planeOrigin + height - radius * height.normalized();
    planeRadius = 2 * radius * height.magnitude() - height.magnitude()*height.magnitude();
    apex = origin + radius * height.normalized();
    openingAngle = asin(planeRadius / radius);

    // // find sideA and sideB
    // // 1. choose arbitrary vector that is not parallel to surfaceNormal
    // Vector x = Vector(1,0,0);
    // if (x.normalized().cross(height.normalized()).magnitude() < 0.01) {
    //     x = Vector(0,1,0);
    // }
    // // 2. find sideA
    // sideA = height.cross(x).normalized();
    // // 3. find sideB
    // sideB = height.cross(sideA).normalized();
}

void PlanoConvex::getBothCollisionTimes(const Ray& ray, double& t_plane, double& t_sphere) const {
    t_plane = calculateCollisionTime(ray.origin, ray.direction, planeOrigin, height.normalized());
    if (t_plane > 0) { 
        Vector p_plane = ray.getPositionAtTime(t_plane);
        // valid position only if close enough to plane origin
        if ((p_plane - planeOrigin).magnitude() > planeRadius) {
            t_plane = Inf;
        }
    } else {
        t_plane = Inf; 
    }
    t_sphere = calculateCollisionTime(ray.origin, ray.direction, origin, radius);
    if (t_sphere > 0) {
        Vector p_sphere = ray.getPositionAtTime(t_sphere);
        // valid position only if close enough to sphere apex
        if (angle(p_sphere-origin, apex-origin) > openingAngle) {
            t_sphere = Inf;
        }
    } else {
        t_sphere = Inf;
    }
}

double PlanoConvex::detectCollisionTime (const Ray& ray) const {
    double t_plane, t_sphere;
    getBothCollisionTimes(ray, t_plane, t_sphere);
    return std::min(t_plane, t_sphere);
}

std::vector<Ray> PlanoConvex::createNewRays (const Ray& ray) const {
    std::vector<Ray> newRays, myNewRays;
    double t_plane, t_sphere;
    getBothCollisionTimes(ray, t_plane, t_sphere);

    if (t_sphere < t_plane) {
        return SphericalLens(origin, radius, refractiveIndex).createNewRays(ray);
    }
    if (t_plane < t_sphere) {
        return ::createNewRays(ray, height.normalized(), refractiveIndex, reflectance);
    }

    return newRays;
}

std::string PlanoConvex::forPythonPlot() const {
    // TODO: rewrite
    std::ostringstream oss;
    oss << "circ = patches.Circle((" << origin.x << ", " << origin.z << "), " << radius << ", alpha=0.05, ec='blue')\n"
    << "ax.add_patch(circ)\n";
    return oss.str();
}

std::vector<Ray> createNewRays (const Ray& ray, Vector surfaceNormal, double n2, double reflectance) {
    // std::vector<Ray> Ray::createReflectionAndRefraction (Vector surfaceNormal, Vector rotationAxis, double n2) {
    /* Create reflection and refraction rays according to Snell's law
    * n1 * sin(theta1) = n2 * sin(theta2)
    * surfaceNormal: direction of surface normal
    * rotationAxis: new rays created based on rotation of old about rotation axis
    * n2: refractive index of other medium (n1 is stored in the Ray object)
    */
    std::vector<Ray> newRays;
    double n1 = ray.refractiveIndex;

    Vector rotationAxis = ray.direction.cross(surfaceNormal).normalized();

    double theta1 = angle(surfaceNormal, ray.direction);
    if (theta1 > M_PI_2) {
        // this happens if the ray is inside the lens!
        rotationAxis = ray.direction.cross(-1*surfaceNormal).normalized();
        theta1 = angle(-1*surfaceNormal, ray.direction);
        n2 = Config::VACUUM_REFRACTIVE_INDEX;
    }
    double theta2 = n1 / n2 * sin(theta1);
    // we create a new direction for the ray by rotating
    // the old direction by theta2-theta1
    double dtheta = theta2 - theta1;
    if (theta1 == 0) {
        newRays.push_back(Ray(ray.end, ray.direction, ray.energyDensity*(1-reflectance), n2));
        newRays.push_back(Ray(ray.end, -1*ray.direction, ray.energyDensity*reflectance, n1));
        return newRays;
    }

    Vector refractionDirection = rotateVectorAboutAxis(ray.direction, rotationAxis, -dtheta);
    newRays.push_back(Ray(ray.end, refractionDirection, ray.energyDensity*(1-reflectance), n2));

    // create reflection
    Vector reflectionDirection = rotateVectorAboutAxis(ray.direction, rotationAxis, -(M_PI-2*theta1));
    newRays.push_back(Ray(ray.end, reflectionDirection, ray.energyDensity*reflectance, n1));
       
    return newRays;
}


//////////////////////////////

ConvexLens::ConvexLens() { }

ConvexLens::ConvexLens(Vector origin_, double radius_, double n_, Vector height_) {
    origin = origin_;
    radius = radius_;
    refractiveIndex = n_;
    height = height_;
    if (height.magnitude() > radius) {
        throw "Height vector must not be longer than sphere radius!";
    }
    sphere1Origin = origin + height - radius * height.normalized();
    sphere2Origin = origin - height + radius * height.normalized();
    apex1 = origin + height;
    apex2 = origin - height;
    double planeRadius = 2 * radius * height.magnitude() - height.magnitude()*height.magnitude();
    // openingAngle = asin(planeRadius / radius);
    openingAngle = acos((radius - height.magnitude())/ radius);
}

void ConvexLens::getBothCollisionTimes(const Ray& ray, double& t1, double& t2) const {
    std::vector<double> times1, times2;
    t1 = Inf; t2 = Inf; // set defaults;
    Vector p;
    times1 = calculateCollisionTimes(ray.origin, ray.direction, sphere1Origin, radius);
    times2 = calculateCollisionTimes(ray.origin, ray.direction, sphere2Origin, radius);


    // test both candidates
    if (times1[0] > Config::MIN_EPS) {
        p = ray.getPositionAtTime(times1[0]);
        if (angle(p-sphere1Origin, apex1-sphere1Origin) < openingAngle) {
            t1 = times1[0];
        }
    } 
    // first candidate did not work out
    if (t1 == Inf) {
        p = ray.getPositionAtTime(times1[1]);
        if (angle(p-sphere1Origin, apex1-sphere1Origin) < openingAngle) {
            t1 = times1[1];
        }        
    }
    // test both candidates
    if (times2[0] > Config::MIN_EPS) {
        p = ray.getPositionAtTime(times2[0]);
        if (angle(p-sphere2Origin, apex2-sphere2Origin) < openingAngle) {
            t2 = times2[0];
        }
    } 
    // first candidate did not work out
    if (t2 == Inf) {
        p = ray.getPositionAtTime(times2[1]);
        if (angle(p-sphere2Origin, apex2-sphere2Origin) < openingAngle) {
            t2 = times2[1];
        }        
    }
    if (t1 < Config::MIN_EPS) { t1 = Inf; }
    if (t2 < Config::MIN_EPS) { t2 = Inf; }

    // edge case: ray hits the intersection of the two spheres: we ignore the collision
    if (abs(t1-t2) < Config::MIN_EPS) {
        t1 = Inf;
        t2 = Inf;
    }
}

double ConvexLens::detectCollisionTime (const Ray& ray) const {
    double t1, t2;
    getBothCollisionTimes(ray, t1, t2);
    return std::min(t1, t2);
}

std::vector<Ray> ConvexLens::createNewRays (const Ray& ray) const {
    std::vector<Ray> newRays, myNewRays;
    double t1, t2;
    getBothCollisionTimes(ray, t1, t2);

    if (t1 < t2) {
        return SphericalLens(sphere1Origin, radius, refractiveIndex).createNewRays(ray);
    }
    if (t2 < t1) {
        return SphericalLens(sphere2Origin, radius, refractiveIndex).createNewRays(ray);
    }

    return newRays;
}

std::string ConvexLens::forPythonPlot() const {
    std::ostringstream oss;
    // oss << "arc1 = Arc((" << sphere1Origin.x << ", " << sphere1Origin.z << "), " << 2*radius << ", " << 2*radius 
    // << ", angle=" << angle(Vector(height.x, 0, height.z), Vector(1,0,0))*180/M_PI << ", theta1=" << 360-1*openingAngle*180/M_PI << ", theta2=" << openingAngle*180/M_PI
    // << ", alpha=0.5, ec='blue')\n"
    // << "ax.add_patch(arc1)\n";
    oss << "circ1 = patchesCircle((" << sphere1Origin.x << ", " << sphere1Origin.z << "), " << radius << ", alpha=0.05, lw=0)\n"
    << "ax.add_patch(circ1)\n";
    oss << "clip1 = patchesCircle((" << sphere2Origin.x << ", " << sphere2Origin.z << "), " << radius << ", alpha=0.05, ec='blue', fill=False, visible=False)\n"
    << "ax.add_patch(clip1)\n";
    oss << "circ1.set_clip_path(clip1)\n";
    // oss << "arc2 = Arc((" << sphere2Origin.x << ", " << sphere2Origin.z << "), " << 2*radius << ", " << 2*radius 
    // << ", angle=" << angle(Vector(-height.x, 0, -height.z), Vector(1,0,0))*180/M_PI << ", theta1=" << 360-1*openingAngle*180/M_PI << ", theta2=" << openingAngle*180/M_PI
    // << ", alpha=0.5, ec='blue')\n"
    // << "ax.add_patch(arc2)\n";
    return oss.str();
}

//////////////////////

ConcaveLens::ConcaveLens(Vector origin_, double radius_, double n_, Vector height_) {
    origin = origin_;
    radius = radius_;
    height = height_;
    refractiveIndex = n_;
    sphere1Origin = origin + height + radius * height.normalized();
    sphere2Origin = origin - height - radius * height.normalized();
    apex1 = origin + height;
    apex2 = origin - height;
    openingAngle = M_PI_2;
}

void ConcaveLens::getBothCollisionTimes(const Ray& ray, double& t1, double& t2) const {
    std::vector<double> times1, times2;
    t1 = Inf; t2 = Inf; // set defaults;
    Vector p;

    times1 = calculateCollisionTimes(ray.origin, ray.direction, sphere1Origin, radius);
    times2 = calculateCollisionTimes(ray.origin, ray.direction, sphere2Origin, radius);

    // test both candidates
    if (times1[0] > Config::MIN_EPS) {
        p = ray.getPositionAtTime(times1[0]);
        if (angle(p-sphere1Origin, apex1-sphere1Origin) < openingAngle) {
            t1 = times1[0];
        }
    } 
    // first candidate did not work out
    if (t1 == Inf) {
        p = ray.getPositionAtTime(times1[1]);
        if (angle(p-sphere1Origin, apex1-sphere1Origin) < openingAngle) {
            t1 = times1[1];
        }        
    }
    // test both candidates
    if (times2[0] > Config::MIN_EPS) {
        p = ray.getPositionAtTime(times2[0]);
        if (angle(p-sphere2Origin, apex2-sphere2Origin) < openingAngle) {
            t2 = times2[0];
        }
    } 
    // first candidate did not work out
    if (t2 == Inf) {
        p = ray.getPositionAtTime(times2[1]);
        if (angle(p-sphere2Origin, apex2-sphere2Origin) < openingAngle) {
            t2 = times2[1];
        }        
    }
    if (t1 < Config::MIN_EPS) { t1 = Inf; }
    if (t2 < Config::MIN_EPS) { t2 = Inf; }

    // edge case: ray hits the intersection of the two spheres: we ignore the collision
    if (abs(t1-t2) < Config::MIN_EPS) {
        t1 = Inf;
        t2 = Inf;
    }
}

double ConcaveLens::detectCollisionTime (const Ray& ray) const {
    double t1, t2;
    getBothCollisionTimes(ray, t1, t2);
    return std::min(t1, t2);
}

std::vector<Ray> ConcaveLens::createNewRays (const Ray& ray) const {
    std::vector<Ray> newRays, myNewRays;
    double t1, t2;
    getBothCollisionTimes(ray, t1, t2);

    if (t1 < t2) {
        // if (ray.refractiveIndex != Config::VACUUM_REFRACTIVE_INDEX) {
        //     return SphericalLens(sphere1Origin, radius, 1/refractiveIndex).createNewRays(ray);
        // }
        return SphericalLens(sphere1Origin, radius, refractiveIndex).createNewRaysInsideOut(ray);
    }
    if (t2 < t1) {
        // if (ray.refractiveIndex != Config::VACUUM_REFRACTIVE_INDEX) {
        //     return SphericalLens(sphere2Origin, radius, 1/refractiveIndex).createNewRays(ray);
        // }
        return SphericalLens(sphere2Origin, radius, refractiveIndex).createNewRaysInsideOut(ray);
    }

    return newRays;
}

std::string ConcaveLens::forPythonPlot() const {
    std::ostringstream oss;

    // assume axis of lens coincides with x axis
    double x0 = std::min(sphere1Origin.x, sphere2Origin.x);
    double z0 = sphere1Origin.z - radius;
    double width = std::abs(sphere1Origin.x - sphere2Origin.x);
    double height = 2 * radius;

    oss << "rect = patches.Rectangle((" << x0 << ", " << z0 << "), width=" << width << ", height=" << height << ", fc='blue', alpha=0.7, zorder=-2)\n"
    << "ax.add_patch(rect)\n"
    << "circle1 = patches.Circle((" << sphere1Origin.x << ", " << sphere1Origin.z << "), radius=" << radius << ", fc='white', ec='none', zorder=-1)\n"
    << "circle2 = patches.Circle((" << sphere2Origin.x << ", " << sphere2Origin.z << "), radius=" << radius << ", fc='white', ec='none', zorder=-1)\n"
    << "ax.add_patch(circle1)\n"
    << "ax.add_patch(circle2)\n";

    return oss.str();
}

////////////////////////

Aperture::Aperture(Vector origin_, Vector surfaceNormal_, double radius_) {
    origin = origin_;
    surfaceNormal = surfaceNormal_.normalized();
    radius = radius_;
}

double Aperture::detectCollisionTime(const Ray& ray) const {
    double t = calculateCollisionTime(ray.origin, ray.direction, origin, surfaceNormal.normalized());
    Vector p = ray.getPositionAtTime(t);
    if ((p-origin).magnitude() < radius) {
        return Inf;
    }
    return t;
}

std::vector<Ray> Aperture::createNewRays (const Ray& ray) const {
    std::vector<Ray> newRays;
    return newRays;
}

std::string Aperture::forPythonPlot() const {
    std::ostringstream oss;
    oss << " ";
    return oss.str();
}


std::ostream& operator<<(std::ostream& os, const SphericalLens& l) {
    os << "Lens: Origin: " << l.origin << " Radius: " << l.radius << " n: " << l.refractiveIndex << "\n";
    return os;
}