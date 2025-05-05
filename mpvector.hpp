#ifndef MPVECTOR_HPP
#define MPVECTOR_HPP

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include "constants.hpp"


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
    Vector operator/(const double scalar);
    Vector cross(const Vector& other) const;
    double dot(const Vector& other);
    double magnitude() const;
    Vector normalized() const;
};

Vector operator*(const double scalar, const Vector& v);

std::ostream& operator<<(std::ostream& os, const Vector& v);

std::vector<double> calculateCollisionTimes(Vector rayOrigin, Vector rayDirection, Vector sphereOrigin, double sphereRadius);
double calculateCollisionTime(Vector rayOrigin, Vector rayDirection, Vector sphereOrigin, double sphereRadius);
double calculateCollisionTime(Vector rayOrigin, Vector rayDirection, Vector planeOrigin, Vector planeNormal);
double calculateCollisionTime(Vector rayOrigin, Vector rayDirection, Vector planeOrigin, Vector planeSideA, Vector planeSideB);
Vector calculateReflectionDirection(Vector rayDirection, Vector surfaceNormal);

// double angle(Vector& a, Vector& b);
double angle(Vector a, Vector b);

Vector rotateVectorAboutAxis(const Vector& v, const Vector& u, double angle);

double mitternacht(double a, double b, double c);


#endif /* MPVECTOR_HPP */