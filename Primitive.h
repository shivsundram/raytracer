#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include "Shape.h"
#include "Transformation.h"
#include "Color.h"
#include "Ray.h"

#include <iostream>
#include <vector>

using namespace std;


void setExtrems(Eigen::Vector4d& currMin, const Eigen::Vector4d& newMin, Eigen::Vector4d& currMax, const Eigen::Vector4d& newMax) {
    for (int i = 0; i < 3; i++) {
        currMin[i] = fmin(currMin[i], newMin[i]);
        currMax[i] = fmax(currMax[i], newMax[i]);
    }
}


class Primitive;

class Material {
public:
    Material();
    Color ambient, specular, diffuse, reflective;
    double specularExponent, refractionCoeff;
    bool isReflective, isDielectric;

    friend ostream& operator<< (ostream &out, Material &m);
};


Material::Material() {
    Color black(0.0, 0.0, 0.0);
    ambient = black;
    specular = black;
    diffuse = black;
    reflective = black;
    specularExponent = 0.0;
    isReflective = false;
    isDielectric = false;
    refractionCoeff = 0.0;
}

ostream& operator<< (ostream &out, Material &m) {
    out << "Ambient " << m.ambient << ", Diffuse " << m.diffuse;
    out << ", Specular " << m.specular << ", Exponent " << m.specularExponent;
    out << ", Reflective " << m.reflective;
    return out;
}



class Intersection {
public:
    const Primitive* primitive;
    LocalGeo local;
};

class Primitive
{
public:
    Primitive() {};
    virtual ~Primitive() = 0;
    virtual bool isHit(const Ray& ray) const = 0;
    virtual Intersection intersect(const Ray& ray) const = 0;
    virtual Material getBRDF() const = 0;
    virtual void getAABB(Eigen::Vector4d& minV, Eigen::Vector4d& maxV) const = 0;
};

inline Primitive::~Primitive() {

}


class GeometricPrimitive : public Primitive
{
private:
    Transformation *objToWorld, *worldToObj;
    Shape *shape;
    Material material;
public:
    GeometricPrimitive(Shape* inShape, Material inMaterial, Transformation* inTransform);
    ~GeometricPrimitive();
    bool isHit(const Ray& ray) const;
    Intersection intersect(const Ray& ray) const;
    Material getBRDF() const { return material; }
    void getAABB(Eigen::Vector4d& minV, Eigen::Vector4d& maxV) const;
};


GeometricPrimitive::GeometricPrimitive(Shape* inShape, Material inMaterial, Transformation* inTransform) {
    shape = inShape;
    material = inMaterial;
    worldToObj = inTransform->getInverse();
    objToWorld = inTransform;
}


GeometricPrimitive::~GeometricPrimitive() {
    delete shape;
    delete worldToObj;
    delete objToWorld;
}


bool GeometricPrimitive::isHit(const Ray &ray) const {
    return shape->isHit(ray);
}


Intersection GeometricPrimitive::intersect(const Ray &ray) const {
    Intersection inter;
    Ray objRay = *worldToObj * ray;

    LocalGeo geo = *objToWorld * shape->intersect(objRay);
    inter.local = geo;

    if (geo.isHit) {
        inter.primitive = this;
    } else {
        inter.primitive = NULL;
    }

    return inter;
}


void GeometricPrimitive::getAABB(Eigen::Vector4d& minV, Eigen::Vector4d& maxV) const {
    vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > corners;
    corners = shape->getAABB();

    double minX = numeric_limits<double>::infinity();
    double minY = minX;
    double minZ = minX;
    double maxX = -numeric_limits<double>::infinity();
    double maxY = maxX;
    double maxZ = maxX;

    // apply objectToWorld transformation and find extremes
    for (int i = 0; i < corners.size(); i++) {
        corners[i] = *objToWorld * corners[i];
        if (corners[i][0] > maxX) {
            maxX = corners[i][0];
        }

        if (corners[i][1] > maxY) {
            maxY = corners[i][1];
        }

        if (corners[i][2] > maxZ) {
            maxZ = corners[i][2];
        }

        if (corners[i][0] < minX) {
            minX = corners[i][0];
        }

        if (corners[i][1] < minY) {
            minY = corners[i][1];
        }

        if (corners[i][2] < minZ) {
            minZ = corners[i][2];
        }
    }

    minV = Eigen::Vector4d(minX, minY, minZ, 1.0);
    maxV = Eigen::Vector4d(maxX, maxY, maxZ, 1.0);
}


class AggregatePrimitive : public Primitive
{

private:
    vector<Primitive*> primitives;
public:
    AggregatePrimitive(vector<Primitive*> &list);
    bool isHit(const Ray& ray) const;
    Intersection intersect(const Ray& ray) const;
    Material getBRDF() const { exit(1); }
    const vector<Primitive*>& getPrimitives() const { return primitives; }
    void getAABB(Eigen::Vector4d& minV, Eigen::Vector4d& maxV) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

AggregatePrimitive::AggregatePrimitive(vector<Primitive*> &list) {
    primitives = list;
}


bool AggregatePrimitive::isHit(const Ray &ray) const {
    for (int i = 0; i < primitives.size(); i++) {
        if (primitives[i]->isHit(ray)) {
            return true;
        }
    }
    return false;
}


Intersection AggregatePrimitive::intersect(const Ray &ray) const {

    double closest_t = numeric_limits<double>::infinity();
    Intersection finalInter, closestInter, intersect;
    bool isPrimitiveHit = false;

    for (int i = 0; i < primitives.size(); i++) {
        intersect = primitives[i]->intersect(ray);

        if (intersect.local.isHit && intersect.local.tHit < closest_t){
            isPrimitiveHit = true;
            closest_t = intersect.local.tHit;
            closestInter = intersect;
        }

    }

    if (isPrimitiveHit) {
        finalInter = closestInter;
    } else {
        finalInter.primitive = NULL;
    }

    return finalInter;
}

void AggregatePrimitive::getAABB(Eigen::Vector4d& minV, Eigen::Vector4d& maxV) const {
    for(int i = 0; i < primitives.size(); i++) {
        Eigen::Vector4d currMin, currMax;
        primitives[i]->getAABB(currMin, currMax);
        setExtrems(minV, currMin, maxV, currMax);
    }
}

#endif
