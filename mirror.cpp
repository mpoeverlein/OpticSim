#include "mirror.hpp"
#include "mpvector.hpp"
#include "optdev.hpp"

// Mirror::Mirror () {
//     // all defaults already set
//     surfaceNormal = sideA.cross(sideB).normalized();
// }

Mirror::Mirror(Vector origin_, Vector sideA_, Vector sideB_, double reflectance_) {
    origin = origin_;
    sideA = sideA_;
    sideB = sideB_;
    surfaceNormal = sideA.cross(sideB).normalized();
    reflectance = reflectance_;
    transmittance = 1 - reflectance;
}

Type Mirror::type() { return Type::Mirror; }

Vector Mirror::getOrigin() { return origin; }
Vector Mirror::getSideA() { return sideA; }
Vector Mirror::getSideB() { return sideB; }
Vector Mirror::getSurfaceNormal() { return surfaceNormal; };
double Mirror::getReflectance() { return reflectance; }

double Mirror::detectCollisionTime(const Ray& ray) const {
    /** the ray meets the mirror at
    *  a * x(t) + b * y(t) + c * z(t) = d
    * see definitions of a,b,c,d in code (define over three points of mirror)
    * We since x,y,z are linear in t, we can solve for t
    */
    Vector p1, p2, p3;
    p1 = origin;
    p2 = p1 + sideA;
    p3 = p1 + sideB;

    double a, b, c, d;
    a = p1.y*p2.z - p2.y*p1.z + p2.y*p3.z - p3.y*p2.z + p3.y*p1.z - p1.y*p3.z;
    b = p1.z*p2.x - p2.z*p1.x + p2.z*p3.x - p3.z*p2.x + p3.z*p1.x - p1.z*p3.x;
    c = p1.x*p2.y - p2.x*p1.y + p2.x*p3.y - p3.x*p2.y + p3.x*p1.y - p1.x*p3.y;
    d = p1.x*p2.y*p3.z - p1.x*p3.y*p2.z + p2.x*p3.y*p1.z - p2.x*p1.y*p3.z + p3.x*p1.y*p2.z - p3.x*p2.y*p1.z;

    Vector mirrorVector(a,b,c);

    // check if ray is parrallel to mirror
    if (mirrorVector.dot(ray.direction) == 0) {
        return Inf; 
    }

    // solve for t and find hitting point
    double t_hit = d / (mirrorVector.dot(ray.direction));
    if (t_hit < 0) { return Inf; }
    Vector p_hit = ray.origin + t_hit * ray.direction;

    // check if ray hits within actual boundaries of mirror
    double a_component, b_component;
    a_component = sideA.normalized().dot(p_hit-origin) / sideA.magnitude();
    b_component = sideB.normalized().dot(p_hit-origin) / sideB.magnitude();
    if ( (a_component > 0) && (a_component < 1) && (b_component > 0) && (b_component < 1) ) {
        return t_hit;
    } else {
        return Inf;
    }
}

std::vector<Ray> Mirror::createNewRays (const Ray& ray) const {
    std::vector<Ray> newRays;
    Vector p_hit = ray.end;
    Vector reflectionDirection;
    if (ray.direction.cross(surfaceNormal).magnitude() == 0) {
        reflectionDirection = -1 * ray.direction;
    } else {
        Vector rotationAxis = ray.direction.cross(surfaceNormal).normalized();
        double theta1 = angle(surfaceNormal, ray.direction);

        // create reflection
        reflectionDirection = rotateVectorAboutAxis(ray.direction, rotationAxis, -(M_PI-2*theta1));
    }

    newRays.push_back(Ray(ray.end, reflectionDirection, ray.energyDensity*reflectance, ray.refractiveIndex));
    return newRays;
}