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

std::ostream& operator<<(std::ostream& os, const Vector& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

/**
 * Return the smallest positive solution for the 2nd degree polynomial
 *   f(x) = a*x^2 + b*x + c.
 * Find x>0 such that f(x) = 0.
 * @param a,b,c 2nd, 1st, 0th coefficients of the polynomial
 * @return Smallest positive solution of the polynomial
 */
double solveSecondDegreePolynomial(double a, double b, double c) {
    if (b*b - 4*a*c < 0) { return Inf; }
    double Delta = sqrt(b*b - 4*a*c);
    double x0 = (-b + Delta) / (2*a);
    double x1 = (-b - Delta) / (2*a);
    if (x1 > Config::MIN_EPS) {
     return x1;
    }
    return x0;
}

/**
 * Return a positive solution for the 2nd degree polynomial
 *   f(x) = a*x^2 + b*x + c.
 * Find x>0 such that f(x) = 0.
 * @param a,b,c 2nd, 1st, 0th coefficients of the polynomial
 * @param returnLarger whether to return the larger solution x or not
 * @return Smallest positive solution of the polynomial
 */
double solveSecondDegreePolynomial(double a, double b, double c, bool returnLarger) {
    if (b*b - 4*a*c < 0) { return Inf; }
    double Delta = sqrt(b*b - 4*a*c);
    double x0 = (-b + Delta) / (2*a);
    double x1 = (-b - Delta) / (2*a);
    if (returnLarger) {
        return x0;
    }
    return x1;
}

/** 
 * Calculate angle in radians between two vectors.
 * @param a,b vectors between which to calculate angle
 * @return angle in radians (i.e. ranges between 0 and pi)
 */
double angle(Vector a, Vector b) {
    return acos(a.dot(b) / (a.magnitude() * b.magnitude()));
}

/**
 * Returns vector that is given by vector v rotated about axis vector u by angle.
 * @param v vector to be rotated
 * @param u vector of axis about which to rotate
 * @param angle rotation angle in radians
 * @return rotated vector
 */
Vector rotateVectorAboutAxis(const Vector& v, const Vector& u, double angle) {
    double S = sin(angle);
    double C = cos(angle);
    double oC = 1 - cos(angle);
    // multiply vector by rotation matrix, here separated by components
    double bx = v.x * (u.x*u.x*oC + C)       + v.y * (u.x*u.y*oC - u.z*S) + v.z * (u.x*u.z*oC + u.y*S);
    double by = v.x * (u.x*u.y*oC + u.z*S)   + v.y * (u.y*u.y*oC + C)     + v.z * (u.y*u.z*oC - u.x*S);
    double bz = v.x * (u.x*u.z*oC - u.y*S)   + v.y * (u.y*u.z*oC + u.x*S) + v.z * (u.z*u.z*oC + C);
    return Vector(bx, by, bz);
}

/**
 * Calculate both collision times for ray interacting with sphere.
 * @param rayOrigin, rayDirection starting vector and direction vector of ray
 * @param sphereOrigin, sphereRadius center point and radius of sphere
 * @return vector<double> of the two collision times
 */
std::vector<double> calculateCollisionTimes(Vector rayOrigin, Vector rayDirection, Vector sphereOrigin, double sphereRadius) {
    Vector o = rayOrigin;
    Vector d = rayDirection;
    Vector c = sphereOrigin;
    double R = sphereRadius;
    Vector v = (c - o);
    double t_p = v.dot(d);

    Vector p = o + t_p * d;
    double dSquared = (p - c).magnitude() * (p - c).magnitude();

    if (dSquared > R * R) {
        return {Inf, Inf};
    } else if (dSquared == R * R) {
        return {Inf, Inf};
    }

    return {solveSecondDegreePolynomial(d.dot(d), -2*d.dot(v), v.dot(v)-R*R, false), 
        solveSecondDegreePolynomial(d.dot(d), -2*d.dot(v), v.dot(v)-R*R, true)};
}

/**
 * Calculate smallest collision time between ray and sphere.
 * @param rayOrigin, rayDirection starting vector and direction vector of ray
 * @param sphereOrigin, sphereRadius center point and radius of sphere
 * @return collision time
 */
double calculateCollisionTime(Vector rayOrigin, Vector rayDirection, Vector sphereOrigin, double sphereRadius) {
    /** 
     * the ray trajectory is given by
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
     * if no collision occurs, the return value is Infinity!
     * 
     */
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
    /**
     * from here: dSquared < R * R, so a hit!
     * hit! create new ray based on first collision
     * we need to find t for
     * R^2 = | o + t*d - c |^2
     *    = | t*d - v |^2
     * the resulting quadratic is
     * (d(dot)d) t^2 - 2t d(dot)v + v(dot)v - R^2 = 0
     * we solve for t
     */
    double t = solveSecondDegreePolynomial(d.dot(d), -2*d.dot(v), v.dot(v)-R*R);
    if (t < 0) { return Inf; }
    return t;   
}

/**
 * Calculate collision time between ray and plane.
 * @param rayOrigin, rayDirection starting vector and direction vector of ray
 * @param planeOrigin, planeNormal origin location of the plane and its surface normal vector
 * @return collision time
 */
double calculateCollisionTime(Vector rayOrigin, Vector rayDirection, Vector planeOrigin, Vector planeNormal) {
    // check if surface and ray are parallel
    if (planeNormal.dot(rayDirection) == 0) { return Inf; }
    return (planeOrigin - rayOrigin).dot(planeNormal) / rayDirection.dot(planeNormal);
}

/**
 * Calculate collision time between ray and a plane that is limited in extent defined by vectors sideA, sideB.
 * @param rayOrigin, rayDirection starting vector and direction vector of ray
 * @param planeOrigin origin location of the plane
 * @param sideA, sideB non-parallel vectors that describe extent of mirror
 * @return collision time. If ray intersects with mirror plane but outside of the region defined by sideA and sideB, Infinity is returned.
 */
double calculateCollisionTime(Vector rayOrigin, Vector rayDirection, Vector planeOrigin, Vector planeSideA, Vector planeSideB) {
    /** 
     * the ray meets the mirror at
     *  a * x(t) + b * y(t) + c * z(t) = d
     * See definitions of a,b,c,d in code (define over three points of mirror)
     * Since x,y,z are linear in t, we can solve for t
     */
    Vector p1, p2, p3;
    p1 = planeOrigin;
    p2 = p1 + planeSideA;
    p3 = p1 + planeSideB;

    double a, b, c, d;
    a = p1.y*p2.z - p2.y*p1.z + p2.y*p3.z - p3.y*p2.z + p3.y*p1.z - p1.y*p3.z;
    b = p1.z*p2.x - p2.z*p1.x + p2.z*p3.x - p3.z*p2.x + p3.z*p1.x - p1.z*p3.x;
    c = p1.x*p2.y - p2.x*p1.y + p2.x*p3.y - p3.x*p2.y + p3.x*p1.y - p1.x*p3.y;
    d = p1.x*p2.y*p3.z - p1.x*p3.y*p2.z + p2.x*p3.y*p1.z - p2.x*p1.y*p3.z + p3.x*p1.y*p2.z - p3.x*p2.y*p1.z;

    Vector mirrorVector(a,b,c);

    // check if ray is parrallel to mirror
    if (mirrorVector.dot(rayDirection) == 0) {
        return Inf; 
    }

    // solve for t and find hitting point
    double t_hit = d / (mirrorVector.dot(rayDirection));
    return t_hit;    
}

/**
 * Calculate new ray direction due to reflection from a surface.
 * Incident angle = Outgoing angle
 * @param rayDirection direction of the ray
 * @param surfaceNormal normal vector of the surface
 * @return direction of the new ray
 */
Vector calculateReflectionDirection(Vector rayDirection, Vector surfaceNormal) {
    if (rayDirection.cross(surfaceNormal).magnitude() == 0) {
        // 90 deg angle between ray and mirror surface
        return -1 * rayDirection;
    }
    Vector rotationAxis = rayDirection.cross(surfaceNormal).normalized();
    double theta1 = angle(surfaceNormal, rayDirection);
    return rotateVectorAboutAxis(rayDirection, rotationAxis, -(M_PI-2*theta1));
}

/**
 * Check if point p is on the spherical dome defined by origin of the sphere, apex of the dome, and opening angle.
 * @param p vector to check
 * @param sphereOrigin origin of the sphere
 * @param apex
 * @param openingAngle
 * @return boolean if point is on dome
 */
bool pointIsOnDome(Vector p, Vector sphereOrigin, Vector apex, double openingAngle) {
    // check if p is on the surface of the sphere
    Vector cp = p - sphereOrigin;
    Vector ca = apex - sphereOrigin;
    // if (cp.magnitude() != ca.magnitude()) { return false; }
    return angle(cp, ca) < openingAngle;
}