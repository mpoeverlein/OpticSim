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
    /** the ray trajectory is given by
    *    r(t) = o + t * d
    * r: position at time t,
    * o: origin of ray,
    * d: unit direction of ray.
    * 
    * the shortest distance between ray and sphere center
    * can be derived from projection. 
    *     t_p = v (dot) d
    * v: vector between sphere origin and ray origin, c - o
    * 
    * if t_p < 0, ray will never collide
    * 
    * The point of the ray closest to the sphere is here:
    *     p = o + t_p * d
    * 
    * Compute squared distance:
    *     D^2 = norm(p - c)^2
    * 
    * Scenarios:
    *     D^2 < R^2: two collisions
    *     D^2 > R^2: miss
    *     D^2 = R^2: exactly one collision
    * 
    * if no collision occurs, the return value is -1!
    * 
    * */ 
   if (ray.energyDensity < Config::MIN_ENERGY_DENSITY) { return Inf; }
   Vector o = ray.origin;
   Vector d = ray.direction;
   Vector c = origin;
   double R = radius;
   Vector v = (c - o);
   double t_p = v.dot(d);
   if (t_p <= Config::MIN_EPS ) {
       return Inf; 
   } 

   Vector p = o + t_p * d;
   double dSquared = (p - c).magnitude() * (p - c).magnitude();

   if (dSquared > R * R) {
       // endT = MAX_T;
       // end = origin + endT * d;
       return Inf;
   } else if (dSquared == R * R) {
       /* ray "touches" sphere
       * what is the interaction here?
       * kept as an individual case for later
       * */
       // endT = MAX_T;
       // end = origin + endT * d;
       return Inf;
   }
   /* from here: dSquared < R * R, so a hit!
   * hit! create new ray based on first collision
   * we need to find t for
   * R^2 = | o + t*d - c |^2
   *    = | t*d - v |^2
   * the resulting quadratic is
   * (d(dot)d) t^2 - 2t d(dot)v + v(dot)v - R^2 = 0
   * we solve for t
   * */
   double t = mitternacht(d.dot(d), -2*d.dot(v), v.dot(v)-R*R);
   if (t < 0) { return Inf; }
   return t; 
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
