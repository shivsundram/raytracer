#ifndef LIGHT_H
#define LIGHT_H

#include <Eigen/Dense>

#include "Color.h"


class Light {
public:
    Light();
    Light(const Color& inColor);
    Color getColor() { return color; }
    virtual Eigen::Vector4d getLightVector(const Eigen::Vector4d& surfacePoint) = 0;
protected:
    Color color;
};

Light::Light() {
    color = Color(0.0, 0.0, 0.0);
}

Light::Light(const Color& inColor) {
    color = inColor;
}



class DLight : public Light {
public:
    DLight();
    DLight(const Color& inColor, const Eigen::Vector4d& inDir) : Light(inColor), direction(inDir) {}
    Eigen::Vector4d getDirection() { return direction; }
    Eigen::Vector4d getLightVector(const Eigen::Vector4d& surfacePoint);
    friend ostream& operator<< (ostream &out, DLight &m);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    Eigen::Vector4d direction;
};

DLight::DLight() {
    direction = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
}


Eigen::Vector4d DLight::getLightVector(const Eigen::Vector4d& surfacePoint) {
    return (-direction).normalized();
}

ostream& operator<< (ostream &out, DLight &l) {
    out << "DLight " << l.color << " " << l.direction;
    return out;
}

//Class for point lights
class PLight : public Light {
public:
    PLight();
    PLight(const Color& inColor, const Eigen::Vector4d& inSource, double inFalloff) : Light(inColor), source(inSource), falloff(inFalloff) {}
    Eigen::Vector4d getSource() { return source; }
    Eigen::Vector4d getLightVector(const Eigen::Vector4d& surfacePoint);
    friend ostream& operator<< (ostream &out, PLight &l);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    Eigen::Vector4d source;
    double falloff;
};

PLight::PLight(){
    source = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
    falloff = 0.0;
}


Eigen::Vector4d PLight::getLightVector(const Eigen::Vector4d& surfacePoint) {
    return (source - surfacePoint).normalized();
}

ostream& operator<< (ostream &out, PLight &l) {
    out << "PLight " << l.color << " " << l.source;
    return out;
}


//Class for ambient lights
class ALight : public Light {
public:
    ALight();
    ALight(const Color& inColor) : Light(inColor) {}
    Eigen::Vector4d getLightVector(const Eigen::Vector4d& surfacePoint);
    friend ostream& operator<< (ostream &out, ALight &l);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

Eigen::Vector4d ALight::getLightVector(const Eigen::Vector4d& surfacePoint) {
    return Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
}

ostream& operator<< (ostream &out, ALight &l) {
    out << "ALight " << l.color;
    return out;
}

#endif