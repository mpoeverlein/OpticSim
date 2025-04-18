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

Vector Vector::operator+(const Vector& other) {
    return {x + other.x, y + other.y, z + other.z};
}

Vector& Vector::operator+=(const Vector& other) {
    x += other.x; y += other.y; z += other.z;
    return *this;
}

Vector Vector::operator-(const Vector& other) {
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

Vector Vector::cross(const Vector& other) {
    return Vector(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

double Vector::dot(const Vector& other) {
    return x * other.x + y * other.y + z * other.z;
}

double Vector::magnitude() {
    return std::sqrt(x*x + y*y + z*z);
}

Vector Vector::normalized() {
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
   if (x1 > MIN_EPS) {
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
