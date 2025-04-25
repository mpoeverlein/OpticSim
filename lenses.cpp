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

std::ostream& operator<<(std::ostream& os, const SphericalLens& l) {
    os << "Lens: Origin: " << l.origin << " Radius: " << l.radius << " n: " << l.refractiveIndex << "\n";
    return os;
}

double SphericalLens::detectCollisionTime(const Ray& ray) const {
   if (ray.energyDensity < Config::MIN_ENERGY_DENSITY) { return Inf; }
   return calculateCollisionTime(ray.origin, ray.direction, origin, radius);
}

std::vector<Ray> SphericalLens::createNewRays (const Ray& ray) const {
    // std::vector<Ray> Ray::createReflectionAndRefraction (Vector surfaceNormal, Vector rotationAxis, double n2) {
    /* Create reflection and refraction rays according to Snell's law
    * n1 * sin(theta1) = n2 * sin(theta2)
    * surfaceNormal: direction of surface normal
    * rotationAxis: new rays created based on rotation of old about rotation axis
    * n2: refractive index of other medium (n1 is stored in the Ray object)
    */
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
    Vector rotationAxis = ray.direction.cross(surfaceNormal).normalized();

    double theta1 = angle(surfaceNormal, ray.direction);
    double theta2 = refractiveIndex / n2 * sin(theta1);
    // we create a new direction for the ray by rotating
    // the old direction by theta2-theta1
    double dtheta = theta2 - theta1;
    Vector refractionDirection = rotateVectorAboutAxis(ray.direction, rotationAxis, -dtheta);
    newRays.push_back(Ray(ray.end, refractionDirection, ray.energyDensity*0.98, n2));

    // create reflection
    Vector reflectionDirection = rotateVectorAboutAxis(ray.direction, rotationAxis, -(M_PI-2*theta1));
    newRays.push_back(Ray(ray.end, reflectionDirection, ray.energyDensity*0.02, refractiveIndex));
    
    return newRays;
}


std::string SphericalLens::forPythonPlot() const {
    std::ostringstream oss;
    oss << "circ = Circle((" << origin.x << ", " << origin.z << "), " << radius << ", alpha=0.05, ec='blue')\n"
    << "ax.add_patch(circ)\n";
    return oss.str();
}

PlanoConvex::PlanoConvex(Vector origin_, double radius_, double n_, Vector height_) {
    origin = origin_;
    radius = radius_;
    refractiveIndex = n_;
    height = height_;
    if (height.magnitude() > radius) {
        throw "Height vector must not be longer than sphere radius!";
    }
    planeOrigin = origin + (radius - height.magnitude()) * height.normalized();
    planeRadius = 2 * radius * height.magnitude() - height.magnitude()*height.magnitude();
    apex = origin + radius * height.normalized();
    openingAngle = acos(planeRadius / radius);

    // find sideA and sideB
    // 1. choose arbitrary vector that is not parallel to surfaceNormal
    Vector x = Vector(1,0,0);
    if (x.normalized().cross(height.normalized()).magnitude() < 0.01) {
        x = Vector(0,1,0);
    }
    // 2. find sideA
    sideA = height.cross(x).normalized();
    // 3. find sideB
    sideB = height.cross(sideA).normalized();
}

double PlanoConvex::detectCollisionTime (const Ray& ray) const {
    double t_plane = calculateCollisionTime(ray.origin, ray.direction, origin, sideA, sideB);
    Vector p_plane = ray.getPositionAtTime(t_plane);
    // valid position only if close enough to plane origin
    if ((p_plane - planeOrigin).magnitude() > planeRadius) {
        t_plane = Inf;
    }
    double t_sphere = calculateCollisionTime(ray.origin, ray.direction, origin, radius);
    Vector p_sphere = ray.getPositionAtTime(t_sphere);
    // valid position only if close enough to sphere apex
    if (angle(p_sphere-origin, apex-origin) > openingAngle) {
        t_sphere = Inf;
    }
    return std::min(t_plane, t_sphere);
}

std::vector<Ray> PlanoConvex::createNewRays (const Ray& ray) const {
    std::vector<Ray> newRays, myNewRays;
    double t_plane = calculateCollisionTime(ray.origin, ray.direction, origin, sideA, sideB);
    Vector p_plane = ray.getPositionAtTime(t_plane);
    // valid position only if close enough to plane origin
    if ((p_plane - planeOrigin).magnitude() > planeRadius) {
        t_plane = Inf;
    }
    double t_sphere = calculateCollisionTime(ray.origin, ray.direction, origin, radius);
    Vector p_sphere = ray.getPositionAtTime(t_sphere);
    // valid position only if close enough to sphere apex
    if (angle(p_sphere-origin, apex-origin) > openingAngle) {
        t_sphere = Inf;
    }
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
    oss << "circ = Circle((" << origin.x << ", " << origin.z << "), " << radius << ", alpha=0.05, ec='blue')\n"
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
    double theta2 = n1 / n2 * sin(theta1);
    // we create a new direction for the ray by rotating
    // the old direction by theta2-theta1
    double dtheta = theta2 - theta1;
    Vector refractionDirection = rotateVectorAboutAxis(ray.direction, rotationAxis, -dtheta);
    newRays.push_back(Ray(ray.end, refractionDirection, ray.energyDensity*(1-reflectance), n2));

    // create reflection
    Vector reflectionDirection = rotateVectorAboutAxis(ray.direction, rotationAxis, -(M_PI-2*theta1));
    newRays.push_back(Ray(ray.end, reflectionDirection, ray.energyDensity*reflectance, n1));
    
    return newRays;
}