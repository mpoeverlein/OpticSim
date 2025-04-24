#include "mpvector.hpp"

Vector::Vector () {
    x = 0.;
    y = 0.;
    z = 0.;
}

Vector::Vector (double x_, double y_, double z_) {
    x = x_;
    y = y_; 
    z = z_;
}

Vector Vector::operator+(const Vector& other) const {
    return {x + other.x, y + other.y, z + other.z};
}

Vector& Vector::operator+=(const Vector& other) {
    x += other.x; y += other.y; z += other.z;
    return *this;
}

Vector Vector::operator-(const Vector& other) const {
    return {x - other.x, y - other.y, z - other.z};
}

Vector& Vector::operator-=(const Vector& other) {
    x -= other.x; y -= other.y; z -= other.z;
    return *this;
}

Vector Vector::operator*(const double scalar) {
    return {scalar * x, scalar * y, scalar * z};
}

Vector& Vector::operator*=(const double scalar) {
    x *= scalar; y *= scalar; z *= scalar;
    return *this;
}

Vector Vector::operator/(const double scalar) {
    return {x/scalar, y/scalar, z/scalar};
}

Vector Vector::cross(const Vector& other) const {
    return Vector(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

double Vector::dot(const Vector& other) {
    return x * other.x + y * other.y + z * other.z;
}

double Vector::magnitude() const {
    return std::sqrt(x*x + y*y + z*z);
}

Vector Vector::normalized() const {
    double mag = magnitude();
    return Vector(x / mag, y / mag, z / mag);
}

Vector operator*(const double scalar, const Vector& v) {
    return {scalar * v.x, scalar * v.y, scalar * v.z};
}

double mitternacht(double a, double b, double c) {
    /* Given a 2nd degree polynomial
    * f(x) = a x^2 + b x + c 
    * find lowest positive number (larger than epsilon to exclude ZERO) for which
    * f(x) = 0
    */
   double Delta = sqrt(b*b - 4*a*c); // always > 0 when we call this
   double x0 = (-b + Delta) / (2*a);
   double x1 = (-b - Delta) / (2*a);
   if (x1 > Config::MIN_EPS) {
    return x1;
   }
   return x0;
}

// double angle(Vector& a, Vector& b) {
//     return acos(a.dot(b) / (a.magnitude() * b.magnitude()));
// }

double angle(Vector a, Vector b) {
    return acos(a.dot(b) / (a.magnitude() * b.magnitude()));
}

Vector rotateVectorAboutAxis(const Vector& v, const Vector& u, double angle) {
    // u: the axis of rotation
    // multiply vector by rotation matrix, here separated by components
    double S = sin(angle);
    double C = cos(angle);
    double oC = 1 - cos(angle);
    double bx = v.x * (u.x*u.x*oC + C)       + v.y * (u.x*u.y*oC - u.z*S) + v.z * (u.x*u.z*oC + u.y*S);
    double by = v.x * (u.x*u.y*oC + u.z*S)   + v.y * (u.y*u.y*oC + C)     + v.z * (u.y*u.z*oC - u.x*S);
    double bz = v.x * (u.x*u.z*oC - u.y*S)   + v.y * (u.y*u.z*oC + u.x*S) + v.z * (u.z*u.z*oC + C);
    return Vector(bx, by, bz);
}

std::ostream& operator<<(std::ostream& os, const Vector& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

double calculateCollisionTime(Vector rayOrigin, Vector rayDirection, Vector sphereOrigin, double sphereRadius) {
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
   Vector o = rayOrigin;
   Vector d = rayDirection;
   Vector c = sphereOrigin;
   double R = sphereRadius;
   Vector v = (c - o);
   double t_p = v.dot(d);
   if (t_p <= Config::MIN_EPS ) {
       return Inf; 
   } 

   Vector p = o + t_p * d;
   double dSquared = (p - c).magnitude() * (p - c).magnitude();

   if (dSquared > R * R) {
       return Inf;
   } else if (dSquared == R * R) {
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