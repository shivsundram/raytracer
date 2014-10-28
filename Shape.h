#ifndef SHAPE_H
#define SHAPE_H

#include <iostream>
#include <math.h>

#include <Eigen/Dense>
#include "Ray.h"

using namespace std;

class Shape
{
public:
    Shape() {};
    virtual ~Shape() = 0;
    virtual bool isHit(const Ray& ray) const = 0;
    virtual LocalGeo intersect(const Ray& ray) const = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


inline Shape::~Shape() { }




class Sphere : public Shape
{
public:
    Sphere();
    Sphere(const Eigen::Vector4f& inOrigin, float inRadius);
    ~Sphere();
    bool isHit(const Ray& ray) const;
    LocalGeo intersect(const Ray& ray) const;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    Eigen::Vector4f origin;
    float radius;
};

Sphere::~Sphere() {
}


Sphere::Sphere() {
    origin = Eigen::Vector4f(0, 0, 0, 1);
    radius = 0.0;
}


Sphere::Sphere(const Eigen::Vector4f& inOrigin, float inRadius) {
    origin = inOrigin;
    radius = inRadius;
}


bool Sphere::isHit(const Ray& r) const {
    Eigen::Vector4f d = r.direction;
    Eigen::Vector4f ec = r.source - origin;

    float term1 = pow(d.dot(ec), 2);
    float term2 = d.dot(d) * (ec.dot(ec) - radius * radius);
    float discr = term1 - term2;

    return discr >= 0;
}

LocalGeo Sphere::intersect(const Ray& ray) const {
    Eigen::Vector4f d = ray.direction;
    Eigen::Vector4f ec = ray.source - origin;

    LocalGeo local;

    float b = d.dot(ec);
    float dItself = d.dot(d);


    float term1 = b * b;
    float term2 = dItself * (ec.dot(ec) - radius * radius);
    float discr = term1 - term2;
    float t1 = (-b - sqrt(discr)) / dItself;

    if (discr < 0.0f || t1<0) {
        local.isHit = false;
    } else {
        
        if (t1 < ray.t_min || t1 > ray.t_max) {
            local.isHit = false;
            return local;
        }
        local.point = ray.source + t1 * d;
        local.normal = local.point - origin;
        local.tHit = t1;
        local.isHit = true;
    }

//    float t2 = (-(d.dot(ec)) + sqrt(discr)) / (d.dot(d));
    return local;
}



class Triangle : public Shape
{
public:
    Triangle();
    Triangle(const Eigen::Vector3f& inVertexA, const Eigen::Vector3f& inVertexB, const Eigen::Vector3f& inVertexC);
    bool isHit(const Ray& ray) const;
    LocalGeo intersect(const Ray& ray) const;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    Eigen::Vector3f vertexA, vertexB, vertexC;
};

Triangle::Triangle(const Eigen::Vector3f& inVertexA, const Eigen::Vector3f& inVertexB, const Eigen::Vector3f& inVertexC){
    vertexA = inVertexA;
    vertexB = inVertexB;
    vertexC = inVertexC;
}

bool Triangle::isHit(const Ray& r) const {
    return false;
}

LocalGeo Triangle::intersect(const Ray& ray) const {

    LocalGeo local;
    Eigen::Vector3f d(ray.direction[0], ray.direction[1], ray.direction[2]);
    Eigen::Vector3f s(ray.source[0], ray.source[1], ray.source[2]);

    Eigen::Vector3f ab = vertexA - vertexB;
    Eigen::Vector3f ac = vertexA - vertexC;
    Eigen::Vector3f as = vertexA - s;
    Eigen::Vector3f n = ab.cross(ac);

    float M = d.dot(n);
    // TODO: check for parallel rays maybe??

    float t = as.dot(n) / M;
    if (t < ray.t_min || t > ray.t_max) {
        local.isHit = false;
        return local;
    }

    Eigen::Vector3f asCrossD = as.cross(d);

    float beta = ac.dot(-asCrossD) / M;
    if (beta < 0 || beta > 1) {
        local.isHit = false;
        return local;
    }

    float gamma = ab.dot(asCrossD) / M;
    if (gamma < 0 || gamma > 1 - beta) {
        local.isHit = false;
        return local;
    }

    local.isHit = true;
    local.point = ray.source + t * ray.direction;
    local.tHit = t;
    local.normal = Eigen::Vector4f(n[0], n[1], n[2], 0.0f);

    return local;
}

#endif
