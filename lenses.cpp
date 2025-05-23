#include "optdev.hpp"
#include "lenses.hpp"
#include "mpvector.hpp"
#include "ray.hpp"
#include "visualizeglfw.hpp"
#include "material.hpp"
#include <sstream>
#include <iomanip>
#include <functional>

Sphere::Sphere(Vector origin_, double radius_)
    : origin(origin_),
      radius(radius_)
{
    if (radius_ == 0) {
        std::cerr << "Sphere radius cannot be zero. Entered value: " << radius_ << "\n";
    }    
}

Sphere::Sphere(const SphereSection& ss) 
        : origin(ss.origin), radius(ss.radius) {}

SphereSection::SphereSection() 
    : origin(Vector()),
      radius(1),
      height(Vector(0,0,1)),  // Normalize height vector
      openingAngle(M_PI)
{

}

SphereSection::SphereSection(Vector origin_, double radius_, Vector height_, double openingAngle_)
    : origin(origin_),
      radius(radius_),
      height(height_.normalized()*radius),  // Normalize height vector
      openingAngle(openingAngle_)
{
    // Validate parameters
    if (radius_ == 0) {
        std::cerr << "SphereSection radius cannot be zero. Entered value: " << radius_ << "\n";
    }
    if (openingAngle_ <= 0 || openingAngle_ > M_PI) {
        std::cerr << "SphereSection Opening angle must be between 0 and π radians. Entered value: " << openingAngle_ << "\n";
    }
    if (height_.magnitude() == 0) {
        std::cerr << "SphereSection Height vector cannot be zero. Entered value: " << height_ << "\n";
    }
}

SphereSection::SphereSection(const Sphere& s)
        : origin(s.origin), radius(s.radius), height(Vector(0,0,s.radius)), openingAngle(M_PI) {}

double SphereSection::detectCollisionTime(const Ray& ray) const {
    return calculateCollisionTime(ray, *this);
}

Vector SphereSection::getSurfaceNormal(const Ray& ray) const {
    return (origin - ray.end).normalized();
}

void SphereSection::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {
    std::cout << "creating vertices" << radius << " opening angle " << openingAngle << "\n";
    std::cout << " for " << origin << " " << height.normalized() << "\n";
    int segments = 16;
    std::vector<Vertex> sphereVerts = createSphereVertices(origin, height.normalized(), abs(radius), openingAngle, segments);
    vertices.insert(vertices.end(), sphereVerts.begin(), sphereVerts.end());

    unsigned int current = 0; 
    if (indices.size() > 0) {
        current = *std::max_element(indices.begin(),indices.end())+1; 
    }
    std::vector<unsigned int> sphereIndices = createSphereIndices(segments, current);
    indices.insert(indices.end(), sphereIndices.begin(), sphereIndices.end());    
}


Lens::Lens(std::vector<std::unique_ptr<SurfaceGeometry>> sgs, 
         std::unique_ptr<Material> m)  // Parameter by value
        : surfaceGeometries(std::move(sgs)), 
          material(std::move(m)) {}

Lens::Lens(Sphere sphere1_, Sphere sphere2_, double refractiveIndex_) {
    Vector o1 = sphere1_.origin, o2 = sphere2_.origin;
    Vector d = o2-o1;
    double x1 = 0, x2 = d.magnitude(), r1 = sphere1_.radius, r2 = sphere2_.radius;
    double xM = 1 / (-2*x2) * (r2*r2 - r1*r1 - x2*x2);
    Vector h1 = d.normalized() * sphere1_.radius;
    Vector h2 = -1 * d.normalized() * sphere2_.radius;
    double a1 = acos((d.magnitude()-xM) / r1);
    double a2 = acos(xM/abs(r2));
    if (r1 < 0) {
        h1 = -1 * h1;
        // a1 = M_PI - a1;
    }
    surfaceGeometries.push_back(std::make_unique<SphereSection>(o1, r1, h1, a1));
    if (r2 > 0) {
        h2 = -1 * h2;
        // a2 = a2;
    }
    surfaceGeometries.push_back(std::make_unique<SphereSection>(o2, r2, h2, a2));
    material = std::make_unique<NonDispersiveMaterial>(refractiveIndex_);

}

std::vector<double> Lens::determineCollisionTimes(const Ray& ray) const {
    std::vector<double> tTimes;
    for (int i = 0; i < surfaceGeometries.size(); i++) {
        tTimes.push_back(surfaceGeometries[i]->detectCollisionTime(ray));
        std::cout << tTimes[i] << "\n";
    }
    return tTimes;
}

double Lens::detectCollisionTime(const Ray& ray) const {
    std::vector<double> tTimes = determineCollisionTimes(ray);
    return *std::min_element(tTimes.begin(), tTimes.end());
}

std::string Lens::forPythonPlot() const { return ""; }

std::vector<Ray> Lens::createNewRays (const Ray& ray) const {
    std::vector<double> tTimes = determineCollisionTimes(ray);
    SurfaceGeometry* sg = surfaceGeometries[std::min_element(tTimes.begin(), tTimes.end())-tTimes.begin()].get();

    std::vector<Ray> newRays;
    Vector surfaceNormal = sg->getSurfaceNormal(ray);
    double otherMedium = material->getRefractiveIndex(ray.wavelength);
    double n2;
    // ray outside or inside
    if (ray.refractiveIndex != otherMedium) {
        n2 = otherMedium;
    } else {
        surfaceNormal = -1 * surfaceNormal;
        n2 = 1.;
    }
    return ::createNewRays(ray, surfaceNormal, n2, 0.1);    
}

void Lens::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {
    for (const auto& sg : surfaceGeometries) {
        sg->createGraphicVertices(vertices, indices);
    }    
}

Lens Lens::makeSphericalLens(Sphere s, std::unique_ptr<Material> m) const {
    Vector h = Vector(0,0,abs(s.radius));
    std::vector<std::unique_ptr<SurfaceGeometry>> sgs;
    sgs.push_back(std::make_unique<SphereSection>(s.origin, s.radius, h, M_PI));
    return Lens(std::move(sgs), std::move(m));
}

SphericalLens::SphericalLens() {}

SphericalLens::SphericalLens(Vector origin_, double radius_, double n_) {
        origin = origin_;
        radius = radius_;
        material = std::make_unique<NonDispersiveMaterial>(n_);
}

SphericalLens::SphericalLens(Vector origin_, double radius_, std::unique_ptr<Material> material)
    : origin(origin_), radius(radius_), material(std::move(material)) {}
      

Vector SphericalLens::getOrigin() { return origin; }
double SphericalLens::getRadius() { return radius; }
double SphericalLens::getRefractiveIndex(double wavelength) { return material->getRefractiveIndex(wavelength); }

/**
 * Calculate collision time between spherical lens and ray as collision time between ray and sphere.
 * @param ray incoming ray
 * @return collision time
 */
double SphericalLens::detectCollisionTime(const Ray& ray) const {
   return calculateCollisionTime(ray.origin, ray.direction, origin, radius);
}

/**
 * Create new rays based on interaction with spherical lens.
 * @param ray incoming ray
 * @return new rays (reflected and refracted rays)
 */
std::vector<Ray> SphericalLens::createNewRays (const Ray& ray) const {
    std::vector<Ray> newRays;
    Vector surfaceNormal;
    double otherMedium = material->getRefractiveIndex(ray.wavelength);
    double n2;
    if (ray.refractiveIndex != otherMedium) {
        // outside
        surfaceNormal = (origin - ray.end).normalized();
        n2 = otherMedium;
    } else {
        // inside
        surfaceNormal = (ray.end - origin).normalized();
        n2 = 1.;
    }

    return ::createNewRays(ray, surfaceNormal, n2, 0.1); // reflectance arbitrarily set to 0.1, should be a dependent on material
}

/**
 * Create rays based on interaction with sphere but for concave lens
 * @param ray incoming ray
 * @return new rays (reflected and refracted rays)
 */
std::vector<Ray> SphericalLens::createNewRaysInsideOut (const Ray& ray) const {
    std::vector<Ray> newRays;
    Vector surfaceNormal;
    double otherMedium = material->getRefractiveIndex(ray.wavelength);
    double n2;
    if (ray.refractiveIndex != otherMedium) {
        // outside
        surfaceNormal = (ray.end - origin).normalized();
        n2 = otherMedium;
    } else {
        // inside
        surfaceNormal = (origin - ray.end).normalized();
        n2 = Config::VACUUM_REFRACTIVE_INDEX;
    }

    return ::createNewRays(ray, surfaceNormal, n2, 0); // reflectance set to 0
}

/**
 * @return string for Python script plotting
 */
std::string SphericalLens::forPythonPlot() const {
    std::ostringstream oss;
    oss << "circ = patches.Circle((" << origin.x << ", " << origin.z << "), " << radius << ", alpha=0.05, ec='blue')\n"
    << "ax.add_patch(circ)\n";
    return oss.str();
}

void SphericalLens::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {
    int segments = 16;
    std::vector<Vertex> sphereVerts = createSphereVertices(origin, Vector(0,0,1), radius, M_PI, segments);
    vertices.insert(vertices.end(), sphereVerts.begin(), sphereVerts.end());

    unsigned int current = 0; 
    if (indices.size() > 0) {
        current = *std::max_element(indices.begin(),indices.end())+1; 
    }
    std::vector<unsigned int> sphereIndices = createSphereIndices(segments, current);
    indices.insert(indices.end(), sphereIndices.begin(), sphereIndices.end());

}

std::ostream& operator<<(std::ostream& os, const SphericalLens& l) {
    os << "Lens: Origin: " << l.origin << " Radius: " << l.radius << " Material: " << l.material << "\n";
    return os;
}


////////////////////////////////

    
PlanoConvex::PlanoConvex(Vector planeOrigin_, double radius_, double n_, Vector height_) {
    planeOrigin = planeOrigin_;
    radius = radius_;
    refractiveIndex = n_;
    height = height_;
    if (height.magnitude() > radius) {
        throw "Height vector must not be longer than sphere radius!";
    }
    origin = planeOrigin + height - radius * height.normalized();
    planeRadius = 2 * radius * height.magnitude() - height.magnitude()*height.magnitude();
    apex = origin + radius * height.normalized();
    openingAngle = asin(planeRadius / radius);
}

/**
 * Calculate collision times of ray for plane and for convex side of the lens.
 * @param ray incoming ray
 * @param[out] t_plane collision time between ray and plane, Infinity if no collision
 * @param[out] t_sphere collision time between ray and sphere, Infinity if no collision
 */
void PlanoConvex::getBothCollisionTimes(const Ray& ray, double& t_plane, double& t_sphere) const {
    t_plane = calculateCollisionTime(ray.origin, ray.direction, planeOrigin, height.normalized());
    if (t_plane > 0) { 
        Vector p_plane = ray.getPositionAtTime(t_plane);
        // valid position only if close enough to plane origin
        if ((p_plane - planeOrigin).magnitude() > planeRadius) {
            t_plane = Inf;
        }
    } else {
        t_plane = Inf; 
    }
    t_sphere = calculateCollisionTime(ray.origin, ray.direction, origin, radius);
    if (t_sphere > 0) {
        Vector p_sphere = ray.getPositionAtTime(t_sphere);
        // valid position only if close enough to sphere apex
        if (angle(p_sphere-origin, apex-origin) > openingAngle) {
            t_sphere = Inf;
        }
    } else {
        t_sphere = Inf;
    }
}

/**
 * Calculate *first* collision time between ray and plano-convex lens.
 * @param ray incoming ray
 * @return collision time, Infinity if no collision
 */
double PlanoConvex::detectCollisionTime (const Ray& ray) const {
    double t_plane, t_sphere;
    getBothCollisionTimes(ray, t_plane, t_sphere);
    return std::min(t_plane, t_sphere);
}

/**
 * Create new rays based on interaction with plano-convex lens.
 * @param ray incoming ray
 * @return vector of new rays
 */
std::vector<Ray> PlanoConvex::createNewRays (const Ray& ray) const {
    std::vector<Ray> newRays, myNewRays;
    double t_plane, t_sphere;
    getBothCollisionTimes(ray, t_plane, t_sphere);

    if (t_sphere < t_plane) {
        return SphericalLens(origin, radius, refractiveIndex).createNewRays(ray);
    }
    if (t_plane < t_sphere) {
        return ::createNewRays(ray, height.normalized(), refractiveIndex, reflectance);
    }

    return newRays;
}

std::string PlanoConvex::forPythonPlot() const {
    // TODO: rewrite
    std::ostringstream oss;
    oss << "circ = patches.Circle((" << origin.x << ", " << origin.z << "), " << radius << ", alpha=0.05, ec='blue')\n"
    << "ax.add_patch(circ)\n";
    return oss.str();
}
void PlanoConvex::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {
    ;
}

/** Create reflection and refraction rays according to Snell's law.
 * @param ray incoming ray
 * @param surfaceNormal normal vector of the surface between current and other medium
 * @param n2 refractive index of the other medium
 * @param reflectance reflectance of the surface
 * @return vector of new rays
 */
std::vector<Ray> createNewRays (const Ray& ray, Vector surfaceNormal, double n2, double reflectance) {
    /* Create reflection and refraction rays according to Snell's law
    * n1 * sin(theta1) = n2 * sin(theta2)
    * surfaceNormal: direction of surface normal
    * rotationAxis: new rays created based on rotation of old about rotation axis
    * n2: refractive index of other medium (n1 is stored in the Ray object)
    */
    std::vector<Ray> newRays;
    double n1 = ray.refractiveIndex;

    Vector rotationAxis = ray.direction.cross(surfaceNormal).normalized();

    double theta1 = angle(surfaceNormal, ray.direction);
    if (theta1 > M_PI_2) {
        // this happens if the ray is inside the lens!
        rotationAxis = ray.direction.cross(-1*surfaceNormal).normalized();
        theta1 = angle(-1*surfaceNormal, ray.direction);
        n2 = Config::VACUUM_REFRACTIVE_INDEX;
    }
    double theta2 = n1 / n2 * sin(theta1);
    // we create a new direction for the ray by rotating
    // the old direction by theta2-theta1
    double dtheta = theta2 - theta1;
    if (theta1 == 0) {
        newRays.push_back(Ray(ray.end, ray.direction, ray.energyDensity*(1-reflectance), n2, ray.wavelength));
        newRays.push_back(Ray(ray.end, -1*ray.direction, ray.energyDensity*reflectance, n1, ray.wavelength));
        return newRays;
    }

    Vector refractionDirection = rotateVectorAboutAxis(ray.direction, rotationAxis, -dtheta);
    newRays.push_back(Ray(ray.end, refractionDirection, ray.energyDensity*(1-reflectance), n2, ray.wavelength));

    // create reflection
    Vector reflectionDirection = rotateVectorAboutAxis(ray.direction, rotationAxis, -(M_PI-2*theta1));
    newRays.push_back(Ray(ray.end, reflectionDirection, ray.energyDensity*reflectance, n1, ray.wavelength));
       
    return newRays;
}


//////////////////////////////

ConvexLens::ConvexLens() { }

ConvexLens::ConvexLens(Vector origin_, double radius_, double n_, Vector height_) {
    origin = origin_;
    radius = radius_;
    refractiveIndex = n_;
    height = height_;
    if (height.magnitude() > radius) {
        throw "Height vector must not be longer than sphere radius!";
    }
    sphere1Origin = origin + height - radius * height.normalized();
    sphere2Origin = origin - height + radius * height.normalized();
    apex1 = origin + height;
    apex2 = origin - height;
    openingAngle = acos((radius - height.magnitude())/ radius);
}

/**
 * Calculate collision times between ray and both sides of the lens.
 * @param ray incoming ray
 * @param[out] t1 collision time with side 1
 * @param[out] t2 collision time with side 2
 */
void ConvexLens::getBothCollisionTimes(const Ray& ray, double& t1, double& t2) const {
    std::vector<double> times1, times2;
    t1 = Inf; t2 = Inf; // set defaults;
    Vector p;
    times1 = calculateCollisionTimes(ray.origin, ray.direction, sphere1Origin, radius);
    times2 = calculateCollisionTimes(ray.origin, ray.direction, sphere2Origin, radius);

    // test both candidates
    if (times1[0] > Config::MIN_EPS) {
        p = ray.getPositionAtTime(times1[0]);
        if (pointIsOnDome(p, sphere1Origin, apex1, openingAngle)) {
            t1 = times1[0];
        }
    } 
    // first candidate did not work out
    if (t1 == Inf) {
        p = ray.getPositionAtTime(times1[1]);
        if (pointIsOnDome(p, sphere1Origin, apex1, openingAngle)) {
            t1 = times1[1];
        }        
    }
    // test both candidates
    if (times2[0] > Config::MIN_EPS) {
        p = ray.getPositionAtTime(times2[0]);
        if (pointIsOnDome(p, sphere2Origin, apex2, openingAngle)) {
            t2 = times2[0];
        }
    } 
    // first candidate did not work out
    if (t2 == Inf) {
        p = ray.getPositionAtTime(times2[1]);
        if (pointIsOnDome(p, sphere2Origin, apex2, openingAngle)) {
            t2 = times2[1];
        }        
    }
    if (t1 < Config::MIN_EPS) { t1 = Inf; }
    if (t2 < Config::MIN_EPS) { t2 = Inf; }

    // edge case: ray hits the intersection of the two spheres: we ignore the collision
    if (abs(t1-t2) < Config::MIN_EPS) {
        t1 = Inf;
        t2 = Inf;
    }
}

/**
 * Calculate first collision time between convex lens and ray.
 * @param ray incoming ray
 * @return first collision time
 */
double ConvexLens::detectCollisionTime (const Ray& ray) const {
    double t1, t2;
    getBothCollisionTimes(ray, t1, t2);
    return std::min(t1, t2);
}

/**
 * Create new rays based on interaction between ray and convex lens.
 * @param ray incoming ray
 * @return vector of new rays based on refraction or reflection with either side of the lens
 */
std::vector<Ray> ConvexLens::createNewRays (const Ray& ray) const {
    std::vector<Ray> newRays, myNewRays;
    double t1, t2;
    getBothCollisionTimes(ray, t1, t2);

    if (t1 < t2) {
        return SphericalLens(sphere1Origin, radius, refractiveIndex).createNewRays(ray);
    }
    if (t2 < t1) {
        return SphericalLens(sphere2Origin, radius, refractiveIndex).createNewRays(ray);
    }

    return newRays;
}

std::string ConvexLens::forPythonPlot() const {
    std::ostringstream oss;
    oss << "circ1 = patches.Circle((" << sphere1Origin.x << ", " << sphere1Origin.z << "), " << radius << ", alpha=0.05, lw=0)\n"
    << "ax.add_patch(circ1)\n";
    oss << "clip1 = patches.Circle((" << sphere2Origin.x << ", " << sphere2Origin.z << "), " << radius << ", alpha=0.05, ec='blue', fill=False, visible=False)\n"
    << "ax.add_patch(clip1)\n";
    oss << "circ1.set_clip_path(clip1)\n";
    return oss.str();
}

void ConvexLens::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {
    int segments = 16;
    unsigned int current = 0; 
    if (indices.size() > 0) {
        current = *std::max_element(indices.begin(),indices.end())+1; 
    }

    std::vector<Vertex> sphereVerts = createSphereVertices(sphere1Origin, height.normalized(), radius, openingAngle, segments);
    vertices.insert(vertices.end(), sphereVerts.begin(), sphereVerts.end());

    std::vector<unsigned int> sphereIndices = createSphereIndices(segments, current);
    indices.insert(indices.end(), sphereIndices.begin(), sphereIndices.end());
    sphereVerts = createSphereVertices(sphere2Origin, -1*height.normalized(), radius, openingAngle, segments);
    vertices.insert(vertices.end(), sphereVerts.begin(), sphereVerts.end());

    current = *std::max_element(indices.begin(),indices.end())+1; 
    sphereIndices = createSphereIndices(segments, current);
    indices.insert(indices.end(), sphereIndices.begin(), sphereIndices.end());
}

//////////////////////

ConcaveLens::ConcaveLens(Vector origin_, double radius_, double n_, Vector height_) {
    origin = origin_;
    radius = radius_;
    height = height_;
    refractiveIndex = n_;
    sphere1Origin = origin + height + radius * height.normalized();
    sphere2Origin = origin - height - radius * height.normalized();
    apex1 = origin + height;
    apex2 = origin - height;
    openingAngle = M_PI_2;
}

/**
 * Calculate collision times between ray and both sides of the lens.
 * @param ray incoming ray
 * @param[out] t1 collision time with side 1
 * @param[out] t2 collision time with side 2
 */
void ConcaveLens::getBothCollisionTimes(const Ray& ray, double& t1, double& t2) const {
    std::vector<double> times1, times2;
    t1 = Inf; t2 = Inf; // set defaults;
    Vector p;

    times1 = calculateCollisionTimes(ray.origin, ray.direction, sphere1Origin, radius);
    times2 = calculateCollisionTimes(ray.origin, ray.direction, sphere2Origin, radius);

    // test both candidates
    if (times1[0] > Config::MIN_EPS) {
        p = ray.getPositionAtTime(times1[0]);
        if (pointIsOnDome(p, sphere1Origin, apex1, openingAngle)) {
            t1 = times1[0];
        }
    } 
    // first candidate did not work out
    if (t1 == Inf) {
        p = ray.getPositionAtTime(times1[1]);
        if (pointIsOnDome(p, sphere1Origin, apex1, openingAngle)) {
            t1 = times1[1];
        }        
    }
    // test both candidates
    if (times2[0] > Config::MIN_EPS) {
        p = ray.getPositionAtTime(times2[0]);
        if (pointIsOnDome(p, sphere2Origin, apex2, openingAngle)) {
            t2 = times2[0];
        }
    } 
    // first candidate did not work out
    if (t2 == Inf) {
        p = ray.getPositionAtTime(times2[1]);
        if (pointIsOnDome(p, sphere2Origin, apex2, openingAngle)) {
            t2 = times2[1];
        }        
    }
    if (t1 < Config::MIN_EPS) { t1 = Inf; }
    if (t2 < Config::MIN_EPS) { t2 = Inf; }

    // edge case: ray hits the intersection of the two spheres: we ignore the collision for now
    if (abs(t1-t2) < Config::MIN_EPS) {
        t1 = Inf;
        t2 = Inf;
    }
}

/** Calculate first collision time
 * @param ray incoming ray
 * @return collision time
 */
double ConcaveLens::detectCollisionTime (const Ray& ray) const {
    double t1, t2;
    getBothCollisionTimes(ray, t1, t2);
    return std::min(t1, t2);
}

/**
 * Create new rays based on interaction between ray and concave lens.
 * @param ray incoming ray
 * @return vector of new rays based on refraction or reflection with either side of the lens
 */
std::vector<Ray> ConcaveLens::createNewRays (const Ray& ray) const {
    std::vector<Ray> newRays, myNewRays;
    double t1, t2;
    getBothCollisionTimes(ray, t1, t2);

    if (t1 < t2) {
        return SphericalLens(sphere1Origin, radius, refractiveIndex).createNewRaysInsideOut(ray);
    }
    if (t2 < t1) {
        return SphericalLens(sphere2Origin, radius, refractiveIndex).createNewRaysInsideOut(ray);
    }

    return newRays;
}

std::string ConcaveLens::forPythonPlot() const {
    std::ostringstream oss;

    // assume axis of lens coincides with x axis
    double x0 = std::min(sphere1Origin.x, sphere2Origin.x);
    double z0 = sphere1Origin.z - radius;
    double width = std::abs(sphere1Origin.x - sphere2Origin.x);
    double height = 2 * radius;

    oss << "rect = patches.Rectangle((" << x0 << ", " << z0 << "), width=" << width << ", height=" << height << ", fc='blue', alpha=0.7, zorder=-2)\n"
    << "ax.add_patch(rect)\n"
    << "circle1 = patches.Circle((" << sphere1Origin.x << ", " << sphere1Origin.z << "), radius=" << radius << ", fc='white', ec='none', zorder=-1)\n"
    << "circle2 = patches.Circle((" << sphere2Origin.x << ", " << sphere2Origin.z << "), radius=" << radius << ", fc='white', ec='none', zorder=-1)\n"
    << "ax.add_patch(circle1)\n"
    << "ax.add_patch(circle2)\n";

    return oss.str();
}

void ConcaveLens::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {

}

////////////////////////

Aperture::Aperture(Vector origin_, Vector surfaceNormal_, double radius_) {
    origin = origin_;
    surfaceNormal = surfaceNormal_.normalized();
    radius = radius_;
}

/**
 * Calculate if ray collides with aperture.
 * @param ray incoming ray
 * @return collision time, Infinity if no collision
 */
double Aperture::detectCollisionTime(const Ray& ray) const {
    double t = calculateCollisionTime(ray.origin, ray.direction, origin, surfaceNormal.normalized());
    Vector p = ray.getPositionAtTime(t);
    if ((p-origin).magnitude() < radius) {
        return Inf;
    }
    return t;
}

/**
 * If ray collides with aperture, the ray is absorbed
 * @param ray incoming ray
 * @return vector of new rays, but it is an empty vector
 */
std::vector<Ray> Aperture::createNewRays ([[maybe_unused]] const Ray& ray) const {
    return std::vector<Ray>();
}

std::string Aperture::forPythonPlot() const {
    std::ostringstream oss;
    oss << " ";
    return oss.str();
}

void Aperture::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {

}

