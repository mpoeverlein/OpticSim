#include "ray.hpp"
#include "mpvector.hpp"
#include <sstream>
#include <iomanip> 
#include "lenses.hpp"
#include "mirror.hpp"
#include "optdev.hpp"

Ray::Ray () {
    origin = Vector();
    direction = Vector();
    energyDensity = 0.0;
    refractiveIndex = 1.;
    wavelength = 550e-9;
}

Ray::Ray (Vector origin_, Vector direction_, double energyDensity_) {
    origin = origin_;
    direction = direction_.normalized();
    energyDensity = energyDensity_;
    refractiveIndex = 1.;
    wavelength = 550e-9;
}

Ray::Ray(Vector origin_, Vector direction_, double energyDensity_, double n) {
    origin = origin_;
    direction = direction_.normalized();
    energyDensity = energyDensity_;
    refractiveIndex = n;
    wavelength = 550e-9;    
}

Ray::Ray(Vector origin_, Vector direction_, double energyDensity_, double n, double wavelength_) {
    origin = origin_;
    direction = direction_.normalized();
    energyDensity = energyDensity_;
    refractiveIndex = n;
    wavelength = wavelength_;     
}

Type Ray::type() {
    return Type::Mirror;
}

std::string Ray::forPythonPlot () {
    std::ostringstream oss;
    std::string color;
    if (refractiveIndex == 1) {
        color = "blue";
    } else {
        color = "orange";
    }
    oss << "ax.plot((" << origin.x << "," << end.x << "), (" 
        << origin.z << "," << end.z << "), linewidth=" 
        << std::setprecision(6) << 2*energyDensity << ", color='" << color << "')\n"; // Adjust precision as needed
    // oss << "ax.scatter(" << origin.x << "," << origin.z << ", marker='o')" << "\n";
    return oss.str();
}

std::vector<Ray> Ray::createReflectionAndRefraction (Vector surfaceNormal, Vector rotationAxis, double n2) {
    /* Create reflection and refraction rays according to Snell's law
    * n1 * sin(theta1) = n2 * sin(theta2)
    * surfaceNormal: direction of surface normal
    * rotationAxis: new rays created based on rotation of old about rotation axis
    * n2: refractive index of other medium (n1 is stored in the Ray object)
    */
    std::vector<Ray> newRays;
    double theta1 = angle(surfaceNormal, direction);
    double theta2 = refractiveIndex / n2 * sin(theta1);
    // we create a new direction for the ray by rotating
    // the old direction by theta2-theta1
    double dtheta = theta2 - theta1;
    Vector refractionDirection = rotateVectorAboutAxis(direction, rotationAxis, -dtheta);
    newRays.push_back(Ray(end, refractionDirection, energyDensity*0.98, n2));

    // create reflection
    Vector reflectionDirection = rotateVectorAboutAxis(direction, rotationAxis, -(M_PI-2*theta1));
    newRays.push_back(Ray(end, reflectionDirection, energyDensity*0.02, refractiveIndex));
    
    return newRays;
}

double Ray::detectCollisionTime (SphericalLens lens) {
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
    if (energyDensity < MIN_ENERGY_DENSITY) { return Inf; }
    Vector o = origin;
    Vector d = direction;
    std::cout << "LENS" << lens << "\n";
    Vector c = lens.getOrigin();
    double R = lens.getRadius();
    Vector v = (c - o);
    double t_p = v.dot(d);
    if (t_p <= MIN_EPS ) {
        endT = MAX_T;
        end = origin + endT * d;
        return Inf; 
    } 

    Vector p = o + t_p * d;
    double dSquared = (p - c).magnitude() * (p - c).magnitude();

    if (dSquared > R * R) {
        // endT = MAX_T;
        // end = origin + endT * d;
        return -1;
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

double Ray::detectCollisionTime(Mirror mirror) {
    /** the ray meets the mirror at
     *  a * x(t) + b * y(t) + c * z(t) = d
     * see definitions of a,b,c,d in code (define over three points of mirror)
     * We since x,y,z are linear in t, we can solve for t
     */
    std::vector<Ray> newRays;
    Vector p1, p2, p3;
    p1 = mirror.getOrigin();
    p2 = mirror.getSideA() + p1;
    p3 = mirror.getSideB() + p1;

    double a, b, c, d;
    a = p1.y*p2.z - p2.y*p1.z + p2.y*p3.z - p3.y*p2.z + p3.y*p1.z - p1.y*p3.z;
    b = p1.z*p2.x - p2.z*p1.x + p2.z*p3.x - p3.z*p2.x + p3.z*p1.x - p1.z*p3.x;
    c = p1.x*p2.y - p2.x*p1.y + p2.x*p3.y - p3.x*p2.y + p3.x*p1.y - p1.x*p3.y;
    d = p1.x*p2.y*p3.z - p1.x*p3.y*p2.z + p2.x*p3.y*p1.z - p2.x*p1.y*p3.z + p3.x*p1.y*p2.z - p3.x*p2.y*p1.z;
    
    Vector mirrorVector(a,b,c);

    // check if ray is parrallel to mirror
    if (mirrorVector.dot(direction) == 0) {
        return Inf; 
    }

    // solve for t and find hitting point
    double t_hit = d / (mirrorVector.dot(direction));
    return t_hit;
}

std::vector<Ray> Ray::createRayFromNewCollision (Mirror mirror) {
    /** the ray meets the mirror at
     *  a * x(t) + b * y(t) + c * z(t) = d
     * see definitions of a,b,c,d in code (define over three points of mirror)
     * We since x,y,z are linear in t, we can solve for t
     */
    std::vector<Ray> newRays;
    Vector p1, p2, p3;
    p1 = mirror.getOrigin();
    p2 = mirror.getSideA() + p1;
    p3 = mirror.getSideB() + p1;

    double a, b, c, d;
    a = p1.y*p2.z - p2.y*p1.z + p2.y*p3.z - p3.y*p2.z + p3.y*p1.z - p1.y*p3.z;
    b = p1.z*p2.x - p2.z*p1.x + p2.z*p3.x - p3.z*p2.x + p3.z*p1.x - p1.z*p3.x;
    c = p1.x*p2.y - p2.x*p1.y + p2.x*p3.y - p3.x*p2.y + p3.x*p1.y - p1.x*p3.y;
    d = p1.x*p2.y*p3.z - p1.x*p3.y*p2.z + p2.x*p3.y*p1.z - p2.x*p1.y*p3.z + p3.x*p1.y*p2.z - p3.x*p2.y*p1.z;
    
    Vector mirrorVector(a,b,c);

    // check if ray is parrallel to mirror
    if (mirrorVector.dot(direction) == 0) {
        return newRays; 
    }

    // solve for t and find hitting point
    double t_hit = d / (mirrorVector.dot(direction));
    Vector p_hit = origin + t_hit * direction;

    // check if p_hit within boundaries of mirror
    // std::cout << "# HIT " << p_hit << "\n";
    double a_component, b_component;
    a_component = mirror.getSideA().normalized().dot(p_hit-mirror.getOrigin()) / mirror.getSideA().magnitude();
    b_component = mirror.getSideB().normalized().dot(p_hit-mirror.getOrigin()) / mirror.getSideB().magnitude();
    // std::cout << p_hit-mirror.getOrigin() << " " << mirror.getSideA() << " " ;
    // std::cout << mirror.getSideA().dot(p_hit-mirror.getOrigin()) << "\n";
    // std::cout << a_component << " " << b_component << "\n";
    if ( (a_component > 0) && (a_component < 1) && (b_component > 0) && (b_component < 1) ) {
        // std::cout << "HIT\n";
        endT = t_hit;
        end = origin + endT * direction;

        // check if ray direction parallel to surfaceNormal
        // if yes, rotationAxis approach does not
        Vector reflectionDirection;
        if (direction.cross(mirror.getSurfaceNormal()).magnitude() == 0) {
            std::cout << "PARA:\n";
            reflectionDirection = -1 * direction;
            std::cout << direction << " " << reflectionDirection << "\n";
        } else {
            Vector rotationAxis = direction.cross(mirror.getSurfaceNormal()).normalized();
            double theta1 = angle(mirror.getSurfaceNormal(), direction);
    
            // create reflection
            reflectionDirection = rotateVectorAboutAxis(direction, rotationAxis, -(M_PI-2*theta1));
        }

        newRays.push_back(Ray(end, reflectionDirection, energyDensity*mirror.getReflectance(), refractiveIndex));
    }

    return newRays;
}

std::vector<Ray> Ray::createRayFromNewCollision (SphericalLens lens) {
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
     * */ 
    std::vector<Ray> newRays;
    // std::cout << "ENERGY DEN " << energyDensity << "\n";
    if (energyDensity < MIN_ENERGY_DENSITY) { return newRays; }
    Vector o = origin;
    Vector d = direction;
    Vector c = lens.getOrigin();
    double R = lens.getRadius();
    Vector v = (c - o);
    double t_p = v.dot(d);
    if (t_p <= MIN_EPS ) {
        endT = MAX_T;
        end = origin + endT * d;
        return newRays; 
    } // empty std::vector<Ray> -> no collision

    Vector p = o + t_p * d;
    double dSquared = (p - c).magnitude() * (p - c).magnitude();

    if (dSquared > R * R) {
        endT = MAX_T;
        end = origin + endT * d;
        return newRays;
    } else if (dSquared == R * R) {
        // ray "touches" sphere
        // what is the interaction here?
        // kept as an individual case for later
        endT = MAX_T;
        end = origin + endT * d;
        return newRays;
    }
    // here: dSquared < R * R, so a hit!
    // hit! create new ray based on first collision
    // we need to find t for
    // R^2 = | o + t*d - c |^2
    //     = | t*d - v |^2
    // the resulting quadratic is
    //  (d(dot)d) t^2 - 2t d(dot)v + v(dot)v - R^2 = 0
    // we solve for t
    double t = mitternacht(d.dot(d), -2*d.dot(v), v.dot(v)-R*R);
    endT = t;
    end = o + endT*d;

    // apply Snell's law to obtain refraction
    // find surface normal, for spherical lens, this is
    // defined by the vector between center and collision (new_origin)
    // check if outside or inside sphere
    // we need to invert the surface normal if inside
    double n2;
    Vector surfaceNormal;
    // Vector rotationAxis;
    if (refractiveIndex != lens.getRefractiveIndex()) {
        // outside
        surfaceNormal = (c - end).normalized();
        n2 = lens.getRefractiveIndex();
    } else {
        // inside
        surfaceNormal = (end - c).normalized();
        n2 = 1.;
    }
    Vector rotationAxis = d.cross(surfaceNormal).normalized();
    // define axis about which to rotate to obtain new direction
    // Vector rotationAxis = d.cross(surfaceNormal).normalized();
    return createReflectionAndRefraction(surfaceNormal, rotationAxis, n2);
}

std::vector<Ray> makeParallelRays(Vector direction, Vector first, Vector last, int steps,
    double energyDensity, double n, double wavelength) {
    std::vector<Ray> rays;
    float ratio;
    Vector final = last - first;
    for (int i = 0; i < steps; i++) {
        ratio = (float) i / steps;
        rays.push_back(Ray(first+ratio*final, direction, energyDensity, n, wavelength));
    }
    return rays;
}

