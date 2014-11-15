#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <vector>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "Ray.h"


#define PI 3.14159265

using namespace std;

class Transformation
{
public:
    Transformation();
    Transformation(const Transformation &rhs);
    Transformation& operator=(const Transformation &rhs);
    friend ostream& operator<< (ostream &out, Transformation &t);
    friend Transformation operator* (const Transformation& x, const Transformation& y);
    friend Eigen::Vector4d operator* (const Transformation& x, const Eigen::Vector4d& y);
    friend Ray operator* (const Transformation& x, const Ray& y);
    friend LocalGeo operator* (const Transformation& x, const LocalGeo& y);

    virtual Transformation* getInverse() const;
    Transformation* compose(const vector<Transformation*> &ts);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
    Eigen::Matrix4d matrix, inverseTranspose;
};


Transformation::Transformation() {
    matrix = Eigen::Matrix4d::Identity();
    inverseTranspose = Eigen::Matrix4d::Identity();
}

Transformation::Transformation(const Transformation &rhs) {
    matrix = rhs.matrix;
    inverseTranspose = rhs.inverseTranspose;
}

Transformation& Transformation::operator=(const Transformation &rhs) {
    if (this == &rhs) {
        return *this;
    }

    matrix = rhs.matrix;
    inverseTranspose = rhs.inverseTranspose;
    return *this;
}


ostream& operator<< (ostream &out, Transformation &t) {
    out << "Matrix: " << endl;
    out << t.matrix;
    out << "Inverse Transpose: " << endl;
    out << t.inverseTranspose;
    return out;
}

Transformation operator* (const Transformation& x, const Transformation& y) {
    Transformation temp;
    temp.matrix = x.matrix * y.matrix;
    temp.inverseTranspose =  x.inverseTranspose * y.inverseTranspose;
    return temp;
}

Ray operator* (const Transformation& x, const Ray& y) {
    Ray temp = y;
    temp.direction = x.matrix * y.direction;
    temp.source = x.matrix * y.source;
    return temp;
}


LocalGeo operator* (const Transformation& x, const LocalGeo& y) {
    LocalGeo temp = y;
    if (y.isHit) {
        temp.normal = x.inverseTranspose * y.normal;
        temp.normal[3] = 0.0;
        temp.normal.normalize();
        temp.point = x.matrix * y.point;
    }
    return temp;
}

Eigen::Vector4d operator* (const Transformation& x, const Eigen::Vector4d& y) {
    return x.matrix * y;
}


Transformation* Transformation::getInverse() const {
    Transformation* temp = new Transformation();
    temp->matrix = (this->inverseTranspose).transpose();
    temp->inverseTranspose = (this->matrix).transpose();
    return temp;
}


Transformation* Transformation::compose(const vector<Transformation*> &ts) {
    Transformation* final = new Transformation();
    for (unsigned i = 0; i < ts.size(); i++ ) {
        *final = *final * *ts[i];
    }

    return final;
}


class Scaling : public Transformation
{
public:
    Scaling(double sx, double sy, double sz);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

Scaling::Scaling(double sx, double sy, double sz) {
    matrix << sx, 0.0, 0.0, 0.0,
              0.0, sy, 0.0, 0.0,
              0.0, 0.0, sz, 0.0,
              0.0, 0.0, 0.0, 1.0;
    inverseTranspose = matrix.inverse().transpose();
}


class Translation : public Transformation
{
public:
    Translation(double tx, double ty, double tz);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


Translation::Translation(double tx, double ty, double tz) {
    matrix << 1.0, 0.0, 0.0, tx,
              0.0, 1.0, 0.0, ty,
              0.0, 0.0, 1.0, tz,
              0.0, 0.0, 0.0, 1.0;
    inverseTranspose = matrix.inverse().transpose();
}


class Rotation : public Transformation
{
public:
    Rotation(double rx, double ry, double rz);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



Rotation::Rotation(double rx, double ry, double rz) {
    if(rx==ry && ry==rz && rz==0){
        return;
    }
    Eigen::Vector3f k(rx, ry, rz);
    double theta = k.norm();
    k.normalize();


    Eigen::Matrix4d ux;
    ux << 0.0, -k[2], k[1], 0.0,
          k[2], 0.0, -k[0], 0.0,
          -k[1], k[0], 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0;


    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();

    double angle = theta*PI/180;

    matrix = I + ux * sin(angle) + ux * ux * (1 - cos(angle));
    matrix(3, 3) = 1.0;
    inverseTranspose = matrix.inverse().transpose();
}

#endif
