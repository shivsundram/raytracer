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
    virtual bool isHit(const Ray& ray) const = 0;
    virtual LocalGeo intersect(const Ray& ray) const = 0;
};


class Sphere : public Shape
{
public:
    Sphere();
    Sphere(const Eigen::Vector4f& inOrigin, float inRadius);
    bool isHit(const Ray& ray) const;
    LocalGeo intersect(const Ray& ray) const;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    Eigen::Vector4f origin;
    float radius;
};


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

    if (discr < 0.0f) {
        local.isHit = false;
        local.tHit = 0.0f;
    } else {
        float t1 = (-b - sqrt(discr)) / dItself;
        local.point = ray.source + t1 * d;
        local.normal = (local.point - origin).normalized();
        local.tHit = t1;
        local.isHit = true;
    }

    // float t2 = (-(d.dot(ec)) + sqrt(discr)) / (d.dot(d));

    return local;
}



class Triangle : public Shape
{
public:
	Triangle();
	Triangle(const Eigen::Vector4f& inVertexA, const Eigen::Vector4f& inVertexB, const Eigen::Vector4f& inVertexC);
	bool isHit(const Ray& ray) const;
	LocalGeo intersect(const Ray& ray) const;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	Eigen::Vector4f vertexA;
	Eigen::Vector4f vertexB;
	Eigen::Vector4f vertexC;
};

Triangle::Triangle(const Eigen::Vector4f& inVertexA, const Eigen::Vector4f& inVertexB, const Eigen::Vector4f& inVertexC){
	vertexA = inVertexA;
	vertexB = inVertexB;
	vertexC = inVertexC; 
}

bool Triangle::isHit(const Ray& r) const {
	return false; 
}

LocalGeo Triangle::intersect(const Ray& ray) const {

	LocalGeo local;
	
	float a = vertexA[0] - vertexB[0];
	float b = vertexA[1] - vertexB[1];;
	float c = vertexA[2] - vertexB[2];;

	float d = vertexA[0] - vertexC[0];
	float e = vertexA[1] - vertexC[1];
	float f = vertexA[2] - vertexC[2];
	
	float g = ray.direction[0]; 
	float h = ray.direction[1];
	float i = ray.direction[2];
	
	float j = vertexA[0] - ray.source[0];
	float k = vertexA[1] - ray.source[1];
	float l = vertexA[2] - ray.source[2];

	float eihf = e*i - h*f;
	float gfdi = g*f - d*i;
	float dheg = d*h - e*g;

	float akjb = a*k - j*b; 
	float jcal = j*c - a*l; 
	float blkc = b*l - k*c;

	float M = a*eihf + b*gfdi + c*dheg;

	float Beta  = ((j*eihf) + (k*gfdi) + (l*dheg)) / M;
	float Gamma = ((i*akjb) + (h*jcal) + (g*blkc)) / M;
	float t = -1 * ((f*akjb) + (e*jcal) + (d*blkc)) / M;


	Eigen::Vector4f V = vertexB - vertexA; 
	Eigen::Vector4f W = vertexC - vertexA;

	local.normal = ray.source + t * ray.direction;

	if ( !((Beta > 0) && (Gamma > 0) && (Beta + Gamma < 1)) ){
		local.isHit = false; 
	}
	else{
		local.isHit = true; 
		local.point = ray.source + t * ray.direction;
		local.tHit = t;

	}
	return local;
}






//class Triangle : public Shape
//{
//public:
//    Triangle();
//    Triangle(const Eigen::Vector4f inA, const Eigen::Vector4f& inB, const Eigen::Vector4f& inC);
//    // include implementation virtual functions for intersection
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//private:
//    Eigen::Vector4f a, b, c;
//};
//
//
//Triangle::Triangle() {
//    Eigen::Vector4f zero(0, 0, 0, 1);
//    a = zero;
//    b = zero;
//    c = zero;
//}
//
//
//Triangle::Triangle(const Eigen::Vector4f& inA, const Eigen::Vector4f& inB, const Eigen::Vector4f& inC) {
//    a = inA;
//    b = inB;
//    c = inC;
//}

#endif
