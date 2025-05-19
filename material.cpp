#include "material.hpp"
#include "mpvector.hpp"
#include <sstream>

NonDispersiveMaterial::NonDispersiveMaterial(double n_) 
    : refractiveIndex(n_) {}

double NonDispersiveMaterial::getRefractiveIndex(double wavelength) const {
    return refractiveIndex;
}

Water::Water(double temperature_) : temperature(temperature_) {}

double Water::getRefractiveIndex(double wavelength) const {
    // model taken from https://www.iapws.org/relguide/rindex.pdf
    // assume density of 1000 kg per m3
    double a0, a1, a2, a3, a4, a5, a6, a7;
    a0 = 0.244257733;
    a1 = 0.00974634476;
    a2 = -0.00373234996;
    a3 = 0.000268678472;
    a4 = 0.0015892057;
    a5 = 0.00245934259;
    a6 = 0.90070492;
    a7 = -0.0166626219;

    double rhoBar = getDensity() / 1000;
    double TBar = temperature / 273.15;
    double lambdaBar = wavelength / 589e-9;
    double lambdaUV = 0.229202;
    double lambdaIR = 5.432937;
    double rhs = 
        a0 + 
        a1*rhoBar + 
        a2*TBar + 
        a3 * lambdaBar*lambdaBar * TBar + 
        a4 / (lambdaBar*lambdaBar) + 
        a5 / (lambdaBar*lambdaBar - lambdaUV*lambdaUV) + 
        a6 / (lambdaBar*lambdaBar - lambdaIR*lambdaIR) +
        a7 * rhoBar * rhoBar;
    rhs *= rhoBar;
    return solveSecondDegreePolynomial(rhs-1, 0, 2*rhs+1);
}

double Water::getDensity() const {
    // return water density at defined temperature in kg per m3
    // this equation uses degrees celsius for a1,a2,a4
    // and degrees celcius squared for a3
    double a1, a2, a3, a4, a5;
    a5 = 999.974950;
    a1 = -3.983035;
    a2 = 301.797;
    a3 = 522528.9;
    a4 = 69.34881;

    return a5 * (1 - 
        (temperature-273.15+a1) * (temperature-273.15+a1) * (temperature-273.15+a2)
        / (a3 * (temperature+273.15+a4)));
}

std::ostream& operator<<(std::ostream& os, const NonDispersiveMaterial& m) {
    os << "Material: Non-dispersive, refractive index " << m.refractiveIndex << "\n";
    return os;
}