#ifndef MPVECTOR_HPP
#define MPVECTOR_HPP

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include "constants.hpp"
#include <glm/glm.hpp>


// Vector contains useful methods for vector mathematics
class Vector
{
    public:
    double x, y, z;
    Vector();
    Vector(double x_, double y_, double z_);
    Vector operator+(const Vector& other) const;
    Vector& operator+=(const Vector& other);
    Vector operator-(const Vector& other) const;
    Vector& operator-=(const Vector& other);
    Vector operator*(const double scalar);
    Vector& operator*=(const double scalar);
    // Vector operator*(const glm::mat3 m);
    Vector operator/(const double scalar);
    Vector cross(const Vector& other) const;
    double dot(const Vector& other) const;
    double magnitude() const;
    Vector normalized() const;
    operator glm::vec3() const {
        return glm::vec3(x, y, z);
    }
};

Vector operator*(const double scalar, const Vector& v);
Vector operator*(const glm::mat3 m, const Vector v);

std::ostream& operator<<(std::ostream& os, const Vector& v);

std::vector<double> calculateCollisionTimes(Vector rayOrigin, Vector rayDirection, Vector sphereOrigin, double sphereRadius);
double calculateCollisionTime(Vector rayOrigin, Vector rayDirection, Vector sphereOrigin, double sphereRadius);
double calculateCollisionTime(Vector rayOrigin, Vector rayDirection, Vector planeOrigin, Vector planeNormal);
void calculateCollisionTime(Vector rayOrigin, Vector rayDirection, Vector planeOrigin, Vector planeSideA, Vector planeSideB, double& t_hit, double& alpha, double& beta);
Vector calculateReflectionDirection(Vector rayDirection, Vector surfaceNormal);
bool pointIsOnDome(Vector p, Vector sphereOrigin, Vector apex, double openingAngle);

// double angle(Vector& a, Vector& b);
double angle(Vector a, Vector b);

Vector rotateVectorAboutAxis(const Vector& v, const Vector& u, double angle);

double solveSecondDegreePolynomial(double a, double b, double c);
double solveSecondDegreePolynomial(double a, double b, double c, bool returnLarger);
double determinant(std::vector<std::vector<double>> mat);
glm::vec3 wavelengthToRGB(double wavelength);
bool operator==(Vector a, Vector b);
bool operator!=(Vector a, Vector b);

class Ray; class SphereSection; class Plane;
double calculateCollisionTime(const Ray& ray, const SphereSection& s);
double calculateCollisionTime(const Ray& ray, const Plane& plane);

#endif /* MPVECTOR_HPP */