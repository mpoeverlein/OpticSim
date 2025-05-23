#ifndef MATERIAL_HPP
#define MATERIAL_HPP

class Material {
    public:
    virtual ~Material() = default;
    virtual double getRefractiveIndex(double wavelength) const = 0;
    virtual double getReflectance(double wavelength) const = 0;
};

class NonDispersiveMaterial : public Material {
    public:
    double refractiveIndex;
    NonDispersiveMaterial(double n_);
    double getRefractiveIndex(double wavelength) const override;
    double getReflectance(double wavelength) const override;
};

class Water : public Material {
    public:
    double temperature;
    Water(double temperature_);
    double getRefractiveIndex(double wavelength) const override;
    double getDensity() const;   
    double getReflectance(double wavelength) const override;
};

#endif /* MATERIAL_HPP */