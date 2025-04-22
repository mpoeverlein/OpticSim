#include "optdev.hpp"
#include "lenses.hpp"
#include "mpvector.hpp"
#include "ray.hpp"
#include <sstream>
#include <iomanip> 

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
   std::cout << "A" << "\n";
   if (ray.energyDensity < MIN_ENERGY_DENSITY) { return Inf; }
   Vector o = ray.origin;
   Vector d = ray.direction;
   Vector c = origin;
   double R = radius;
   Vector v = (c - o);
   double t_p = v.dot(d);
   if (t_p <= MIN_EPS ) {
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
   return t; 
}