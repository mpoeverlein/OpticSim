#include "mpvector.hpp"
#include "lenses.hpp"

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

Vector operator*(const glm::mat3 m, const Vector v) {
    double x = m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z;
    double y = m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z;
    double z = m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z;
    return Vector(x,y,z);
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

double calculateCollisionTime(const Ray& ray, const SphereSection& s) {
    Vector o = ray.origin;
    Vector d = ray.direction;
    Vector c = s.origin;
    double R = s.radius;
    Vector v = (c - o);
    double t_p = v.dot(d);

    Vector p = o + t_p * d;
    double dSquared = (p - c).magnitude() * (p - c).magnitude();

    if (dSquared > R * R) {
        return Inf;
    } else if (dSquared == R * R) {
        return Inf;
    }

    double a1 = d.dot(d), a2 = -2*d.dot(v), a3 = v.dot(v)-R*R;

    double t = (-a2 - sqrt(a2*a2 - 4*a1*a3)) / (2*a1);
    if (t <= 0) { 
        t = Inf; 
    } else {
        if (angle(ray.getPositionAtTime(t)-c, s.height-c) > s.openingAngle) { t = Inf; }
    } 
    if (t != Inf) { return t; }
    t = (-a2 + sqrt(a2*a2 - 4*a1*a3)) / (2*a1);
    if (t <= 0) { 
        t = Inf; 
    } else {
        if (angle(ray.getPositionAtTime(t)-c, s.height-c) > s.openingAngle) { t = Inf; }
    }
    return t;
}

/**
 * Calculate smallest collision time between ray and sphere.
 * @param rayOrigin, rayDirection starting vector and direction vector of ray
 * @param sphereOrigin, sphereRadius center point and radius of sphere
 * @return collision time
 */
double calculateCollisionTime(Vector rayOrigin, Vector rayDirection, Vector sphereOrigin, double sphereRadius) {
    // mathematical derivation in mathematics.md
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
 * @param[out] t_hit, alpha, beta collision time and coefficients for sideA and sideB
 * @return collision time. If ray intersects with mirror plane but outside of the region defined by sideA and sideB, Infinity is returned.
 */
void calculateCollisionTime(
    Vector rayOrigin, Vector rayDirection, 
    Vector planeOrigin, Vector planeSideA, Vector planeSideB, 
    double& t_hit, double& alpha, double& beta) {
    // mathematical derivation in mathematics.md

    std::vector<std::vector<double>> coeff = {
        {rayDirection.x, -planeSideA.x, -planeSideB.x},
        {rayDirection.y, -planeSideA.y, -planeSideB.y},
        {rayDirection.z, -planeSideA.z, -planeSideB.z}
    };
    std::vector<double> constants = {planeOrigin.x-rayOrigin.x, planeOrigin.y-rayOrigin.y, planeOrigin.z-rayOrigin.z};

    double D = determinant(coeff);
    // no collision
    if (abs(D) < 1e-9) { 
        t_hit = Inf; 
        return;
    } 

    // use Cramer's rule
    std::vector<std::vector<double>> matT = coeff, matAlpha = coeff, matBeta = coeff;
    for (int i = 0; i < 3; ++i) {
        matT[i][0] = constants[i];  // Replace 1st column
        matAlpha[i][1] = constants[i];
        matBeta[i][2] = constants[i];
    }

    double DT = determinant(matT);
    double DAlpha = determinant(matAlpha);
    double DBeta = determinant(matBeta);

    t_hit = DT / D;
    alpha = DAlpha / D;
    beta = DBeta / D;
}

/**
 * Calculate new ray direction due to reflection from a surface.
 * Incident angle = Outgoing angle
 * @param rayDirection direction of the ray
 * @param surfaceNormal normal vector of the surface
 * @return direction of the new ray
 */
Vector calculateReflectionDirection(Vector rayDirection, Vector surfaceNormal) {
    return rayDirection - 2 * (rayDirection.dot(surfaceNormal)) * surfaceNormal;
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

/**
 * Calculate determinant of a 3x3 matrix
 */
double determinant(std::vector<std::vector<double>> mat) {
    return mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) -
           mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) +
           mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
}

glm::vec3 wavelengthToRGB(double wavelength) {
    float r, g, b;
    // No clamping as to not change input 
    // // Only consider visible spectrum (380-780 nm)
    // wavelength = std::clamp(wavelength, 380.0, 780.0);
    // wavelength input in meters, so convert to nanometers
    wavelength *= 1e9;
    
    // Calculate color based on wavelength
    if (wavelength >= 380 && wavelength < 440) {
        r = -(wavelength - 440) / (440 - 380);
        g = 0.0;
        b = 1.0;
    } 
    else if (wavelength >= 440 && wavelength < 490) {
        r = 0.0;
        g = (wavelength - 440) / (490 - 440);
        b = 1.0;
    } 
    else if (wavelength >= 490 && wavelength < 510) {
        r = 0.0;
        g = 1.0;
        b = -(wavelength - 510) / (510 - 490);
    } 
    else if (wavelength >= 510 && wavelength < 580) {
        r = (wavelength - 510) / (580 - 510);
        g = 1.0;
        b = 0.0;
    } 
    else if (wavelength >= 580 && wavelength < 645) {
        r = 1.0;
        g = -(wavelength - 645) / (645 - 580);
        b = 0.0;
    } 
    else {
        r = 1.0;
        g = 0.0;
        b = 0.0;
    }
    
    // Let the intensity fall off near the vision limits
    double factor;
    if (wavelength >= 380 && wavelength < 420) {
        factor = 0.3 + 0.7 * (wavelength - 380) / (420 - 380);
    } 
    else if (wavelength >= 420 && wavelength < 700) {
        factor = 1.0;
    } 
    else {
        factor = 0.3 + 0.7 * (780 - wavelength) / (780 - 700);
    }
    
    // Apply intensity factor
    r = std::clamp(std::pow(r * factor, 0.8), 0.0, 1.0);
    g = std::clamp(std::pow(g * factor, 0.8), 0.0, 1.0);
    b = std::clamp(std::pow(b * factor, 0.8), 0.0, 1.0);
    
    return glm::vec3(r,g,b);
}

bool operator==(Vector a, Vector b) {
    constexpr double epsilon = 1e-10;
    return std::abs(a.x - b.x) < epsilon &&
        std::abs(a.y - b.y) < epsilon &&
        std::abs(a.z - b.z) < epsilon;
}

bool operator!=(Vector a, Vector b) {
    return !(a == b);
}