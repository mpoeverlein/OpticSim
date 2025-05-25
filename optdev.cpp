#include "optdev.hpp"
#include "surfacegeometry.hpp"
#include "material.hpp"
#include "ray.hpp"
#include "mpvector.hpp"
#include "visualizeglfw.hpp"
#include <sstream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

Lens::Lens() {}

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
    }
    surfaceGeometries.push_back(std::make_unique<SphereSection>(o1, r1, h1, a1));
    if (r2 > 0) {
        h2 = -1 * h2;
    }
    surfaceGeometries.push_back(std::make_unique<SphereSection>(o2, r2, h2, a2));
    material = std::make_unique<NonDispersiveMaterial>(refractiveIndex_);

}

std::vector<double> Lens::determineCollisionTimes(const Ray& ray) const {
    std::vector<double> tTimes;
    for (int i = 0; i < surfaceGeometries.size(); i++) {
        tTimes.push_back(surfaceGeometries[i]->detectCollisionTime(ray));
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
    std::cout << "surface normal " << surfaceNormal << "\n";
    return ::createNewRays(ray, surfaceNormal, n2, material->getReflectance(ray.wavelength));    
}

void Lens::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {
    for (const auto& sg : surfaceGeometries) {
        sg->createGraphicVertices(vertices, indices);
    }    
}

Lens Lens::makeSphericalLens(Sphere s, std::unique_ptr<Material> m) {
    Vector h = Vector(0,0,abs(s.radius));
    std::vector<std::unique_ptr<SurfaceGeometry>> sgs;
    sgs.push_back(std::make_unique<SphereSection>(s.origin, s.radius, h, M_PI));
    return Lens(std::move(sgs), std::move(m));
}

Lens Lens::makeConvexLens(Vector origin_, double radius_, Vector height_, std::unique_ptr<Material> m) {
    if (radius_ <= 0) { std::cerr << "Radius to make symmetric convex lens must be positve. Entered value: " << radius_ << "\n"; }
    std::vector<std::unique_ptr<SurfaceGeometry>> sgs;
    Vector o1 = origin_ + height_ - height_.normalized()*radius_;
    double a1 = acos((radius_ - height_.magnitude()) / radius_);
    sgs.push_back(std::make_unique<SphereSection>(o1, radius_, height_.normalized(), a1));

    Vector o2 = origin_ - height_ + height_.normalized()*radius_;
    sgs.push_back(std::make_unique<SphereSection>(o2, radius_, -1*height_.normalized(), a1));


    return Lens(std::move(sgs), std::move(m));
}


std::string Lens::toString() const {
    std::string result;
    for (const auto& sg : surfaceGeometries) {
        result += sg->toString();
    }
    return result;
}

void Lens::setTransverseRadius(double newRadius) {
    if (surfaceGeometries.size() != 2) { 
        std::cerr << "To set transverse radius, the system must comprise two surface geometries.\n";
        std::cerr << "The current object has: " << surfaceGeometries.size() << " surface geometries.\n";
        return;
    }
    double transverseRadius1, transverseRadius2;
    SphereSection* ss1 = dynamic_cast<SphereSection*>(surfaceGeometries[0].get());
    if (ss1) { transverseRadius1 = abs(ss1->radius) * sin(ss1->openingAngle); }
    SphereSection* ss2 = dynamic_cast<SphereSection*>(surfaceGeometries[1].get());
    if (ss2) { transverseRadius2 = abs(ss2->radius) * sin(ss2->openingAngle); }
    std::cout << "tr1 " << transverseRadius1 << " tr2 " << transverseRadius2 << "\n";
    if (transverseRadius1 != transverseRadius2) {
        std::cerr << "Transverse radii should match. Entered values: " << transverseRadius1 << ", " << transverseRadius2 << "\n";
        return;
    }
    if (newRadius >= transverseRadius1) {
        std::cerr << "New radius must be smaller than current radius: " << newRadius << ", " << transverseRadius1 << "\n";
        return;
    }
    ss1->openingAngle = asin(newRadius/ss1->radius);
    ss2->openingAngle = asin(newRadius/ss2->radius);

    // add cylinder
    Vector csOrigin = ss1->origin + ss1->height.normalized() * cos(ss1->openingAngle);
    Vector csEnd = ss2->origin + ss2->height.normalized() * cos(ss2->openingAngle);
    Vector csHeight = csEnd - csOrigin;
    CylinderSide cs{csOrigin, csHeight, newRadius};
    surfaceGeometries.push_back(std::make_unique<CylinderSide>(std::move(cs)));
}

Lens Lens::makePlanoConvexLens(Vector origin_, double radius_, Vector height_, std::unique_ptr<Material> m) {
    std::vector<std::unique_ptr<SurfaceGeometry>> sgs;
    Vector o1 = origin_ + height_ - height_.normalized()*radius_;
    double a1 = acos((radius_ - height_.magnitude()) / radius_);
    sgs.push_back(std::make_unique<SphereSection>(o1, radius_, height_.normalized(), a1));

    Vector o = origin_ - height_ + height_.normalized()*radius_;
    double discRadius = radius_ * sin(a1);
    sgs.push_back(std::make_unique<Disc>(origin_, -1*height_.normalized(), discRadius));

    return Lens(std::move(sgs), std::move(m));    
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
    Plane tempPlane{origin, surfaceNormal};
    double t = calculateCollisionTime(ray, tempPlane);
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

Mirror::Mirror () {
    origin = Vector();
    sideA = Vector(1,0,0);
    sideB = Vector(0,1,0);
    surfaceNormal = sideA.cross(sideB).normalized();
    reflectance = 1;
    transmittance = 0;    
 }

Mirror::Mirror(Vector origin_, Vector sideA_, Vector sideB_, double reflectance_) {
    origin = origin_;
    sideA = sideA_;
    sideB = sideB_;
    surfaceNormal = sideA.cross(sideB).normalized();
    reflectance = reflectance_;
    transmittance = 1 - reflectance;
}

double Mirror::detectCollisionTime(const Ray& ray) const {
    double t_hit = Inf;
    double alpha = Inf;
    double beta = Inf;
    calculateCollisionTime(ray.origin, ray.direction, origin, sideA, sideB, t_hit, alpha, beta);
    if (t_hit < Config::MIN_EPS) { return Inf; }
    if (t_hit == Inf) { return Inf; }

    if ( (alpha > 0) && (alpha < 1) && (beta > 0) && (beta < 1) ) {
        return t_hit;
    } else {
        return Inf;
    }
}

std::vector<Ray> Mirror::createNewRays (const Ray& ray) const {
    std::vector<Ray> newRays;
    Vector reflectionDirection = calculateReflectionDirection(ray.direction, surfaceNormal);
    // reflected ray
    newRays.push_back(Ray(ray.end, reflectionDirection, ray.energyDensity*reflectance, ray.refractiveIndex));
    // transmitted ray
    newRays.push_back(Ray(ray.end, ray.direction, ray.energyDensity*(1-reflectance), ray.refractiveIndex));
    return newRays;
}

std::string Mirror::forPythonPlot() const {
    std::ostringstream oss;
    oss << "ax.plot((" << origin.x << "," << origin.x+sideA.x << "), (" 
    << origin.z << "," << origin.z+sideA.z <<  "), linewidth=1, color='k')\n";
    oss << "ax.plot((" << origin.x << "," << origin.x+sideB.x << "), (" 
    << origin.z << "," << origin.z+sideB.z <<  "), linewidth=1, color='k')\n";
    return oss.str();
}

void Mirror::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {
    // vertices.push_back(Vertex{glm::vec3(origin.x,origin.y,origin.z), glm::vec3(surfaceNormal.x,surfaceNormal.y,surfaceNormal.z), glm::vec3(1.0,0,0), 0.4f});
    glm::vec3 color{1,0,0};
    float opacity = 0.4;
    vertices.push_back(Vertex{glm::vec3(origin), glm::vec3(surfaceNormal), color, opacity});
    vertices.push_back(Vertex{glm::vec3(origin+sideA), glm::vec3(surfaceNormal), color, opacity});
    vertices.push_back(Vertex{glm::vec3(origin+sideB), glm::vec3(surfaceNormal), color, opacity});
    vertices.push_back(Vertex{glm::vec3(origin+sideA+sideB), glm::vec3(surfaceNormal), color, opacity});
    unsigned int firstIndex = 0;
    if (indices.size() > 0) {
        firstIndex = *std::max_element(indices.begin(), indices.end()) + 1;
    }
    indices.insert(indices.end(), {firstIndex, firstIndex+1, firstIndex+2});
    indices.insert(indices.end(), {firstIndex+1, firstIndex+3, firstIndex+2});

}


std::string Mirror::toString() const {
    return "Mirror "; 
}


//////////

ParabolicMirror::ParabolicMirror() {
    ;
}

ParabolicMirror::ParabolicMirror(Vector origin_, Vector height_, double curvature_) {
    origin = origin_;
    height = height_;
    curvature = curvature_;
    focalPoint = origin + 1 / (4*curvature) * height.normalized();
}

ParabolicMirror::ParabolicMirror(Vector origin_, Vector height_, Vector focalPoint_) {
    // origin = origin_;
    // height = height_;
    // focalPoint = focalPoint_;
    // curvature = 
}

ParabolicMirror::ParabolicMirror(Vector origin_, Vector height_, double curvature_, double reflectance_) {
    origin = origin_;
    height = height_;
    curvature = curvature_;
    reflectance = reflectance_;
    focalPoint = origin + 1 / (4*curvature) * height.normalized();
}

glm::mat3 ParabolicMirror::getRotationMatrixForLocalCoordinates() const {
    const Vector Z(0.0f, 0.0f, 1.0f);
    Vector axis = height.normalized().cross(Z);
    if (axis.magnitude() < 1e-6f) {
        axis = Vector(1.0f, 0.0f, 0.0f); // Handle parallel case
    }
    axis = axis.normalized();
    float rotationAngle = angle(Z, height);
    // std::cout << "# AXIS " << axis << " angle " << rotationAngle*180/M_PI << "\n";
    return glm::mat3(glm::rotate(glm::mat4(1.0f), rotationAngle, glm::vec3(axis)));
}

double ParabolicMirror::detectCollisionTime(const Ray& ray) const {
    double t;
    // transform mirror and ray such that mirror is defined by k*(x^2+y^2) = z.
    // 1. Compute rotation matrix
    glm::mat3 R = getRotationMatrixForLocalCoordinates();

    // 2. Transform ray to parabola's local space
    Vector o_local = glm::transpose(R) * (ray.origin - origin);
    Vector d_local = glm::transpose(R) * ray.direction;
    // std::cout << "# MY DLOCA " << d_local << "\n";
    // std::cout << "# DLCO x Z " <<  d_local.cross(Vector(0,0,1)).magnitude() << "\n";
    // std::cout << "# DLCO dot Z " << d_local.normalized().dot(Vector(0,0,1)) << "\n";

    // edge case: ray is parallel to mirror's height vector, z value in local coordinates is then given by x and y of o_local
    // if (d_local.cross(Vector(0,0,1)).magnitude() < 1e-8) {
    if (1 - abs(d_local.normalized().dot(Vector(0,0,1))) < 1e-4) {
        double z_hit = curvature*(o_local.x*o_local.x + o_local.y*o_local.y);
        if (z_hit > height.magnitude()) { return Inf; }
        t = (z_hit-o_local.z) / d_local.z;
        if (t < Config::MIN_EPS) {
            return Inf;
        }
        return t;
    }

    // 3. Compute quadratic coefficients
    double A = curvature * (d_local.x * d_local.x + d_local.y * d_local.y);
    double B = 2.0f * curvature * (o_local.x * d_local.x + o_local.y * d_local.y) - d_local.z;
    double C = curvature * (o_local.x * o_local.x + o_local.y * o_local.y) - o_local.z;

    // 4. Solve quadratic
    double discriminant = B * B - 4 * A * C;
    if (discriminant < 0) return Inf; // No intersection
    // std::cout << "# DISC " << discriminant << "\n";
    // std::cout << "# A " << A << "\n";

    t = (-B - sqrt(discriminant)) / (2 * A);
    if ((o_local + t * d_local).z > height.magnitude()) { t = 0; }

    if (t < Config::MIN_EPS) {
        t = (-B + sqrt(discriminant)) / (2 * A); // Try other solution
        if ((o_local + t * d_local).z > height.magnitude()) { t = Inf; }
        if (t < Config::MIN_EPS) { t = Inf; } // Both solutions behind ray
        if (t > Config::MAX_T) { t = Inf; }
    }
    // std::cout << "# T " << t << "\n";
    return t;
}
std::vector<Ray> ParabolicMirror::createNewRays(const Ray& ray) const {
    std::vector<Ray> newRays;
    // surface normal in local coordinates given by gradient
    glm::mat3 R = getRotationMatrixForLocalCoordinates();
    Vector end_local = glm::transpose(R) * (ray.end - origin);
    Vector d_local = glm::transpose(R) * ray.direction;
    Vector surfaceNormal;

    surfaceNormal = -1 * Vector(2*curvature*end_local.x, 2*curvature*end_local.y, -1).normalized();
    Vector reflectionDirection = calculateReflectionDirection(d_local, surfaceNormal);
    reflectionDirection = R * reflectionDirection;
    newRays.push_back(Ray(ray.end, reflectionDirection, ray.energyDensity*reflectance, ray.refractiveIndex));
    return newRays;
}
std::string ParabolicMirror::forPythonPlot() const {
    return "";
}
void ParabolicMirror::createGraphicVertices(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) const {
    int segments = 16;
    std::vector<Vertex> parabolaVerts = createParabolaVertices(origin, height, curvature, segments, glm::vec3(1,0.5,0.5));
    vertices.insert(vertices.end(), parabolaVerts.begin(), parabolaVerts.end());

    unsigned int current = 0; 
    if (indices.size() > 0) {
        current = *std::max_element(indices.begin(),indices.end())+1; 
    }
    std::vector<unsigned int> parabolaIndices = createSphereIndices(segments, current);
    indices.insert(indices.end(), parabolaIndices.begin(), parabolaIndices.end());
}

std::string ParabolicMirror::toString() const {
    return "Parabolic Mirror "; 
}