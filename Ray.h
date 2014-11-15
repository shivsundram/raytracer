#ifndef RAY_H
#define RAY_H

#include <limits>
#include <Eigen/Dense>

using namespace std;

class Ray {
public:
    Eigen::Vector4d source;
    Eigen::Vector4d direction;
    double t_min, t_max;
    Ray();
    Ray(const Eigen::Vector4d& inSource, const Eigen::Vector4d& inDirection, double tMinIn, double tMaxIn);
    Ray(const Ray &rhs);
    Ray& operator=(const Ray &rhs);
    friend ostream& operator<< (ostream &out, Ray &r);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

Ray::Ray() {
    source = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
    direction = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
    t_min = -numeric_limits<double>::infinity();
    t_max = numeric_limits<double>::infinity();
}

Ray::Ray(const Eigen::Vector4d& inSource, const Eigen::Vector4d& inDirection, double tMinIn, double tMaxIn) {
    source = inSource;
    direction = inDirection;
    t_min = tMinIn;
    t_max = tMaxIn;
}

Ray::Ray(const Ray &rhs) {
    source = rhs.source;
    direction = rhs.direction;
    t_min = rhs.t_min;
    t_max = rhs.t_max;
}

Ray& Ray::operator=(const Ray &rhs) {
    if (this == &rhs) {
        return *this;
    }

    source = rhs.source;
    direction = rhs.direction;
    t_min = rhs.t_min;
    t_max = rhs.t_max;
    return *this;
}

ostream& operator<< (ostream &out, Ray &r) {
    out << "Source " << r.source << ", Direction " << r.direction;
    return out;

}


// Let's wait and see if we need this
class LocalGeo {
public:
    Eigen::Vector4d point;
    Eigen::Vector4d normal;
    double tHit;
    bool isHit;
    LocalGeo();
    LocalGeo(const LocalGeo &rhs);
    LocalGeo& operator=(const LocalGeo &rhs);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


LocalGeo::LocalGeo() {
    point = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
    normal = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
    tHit = numeric_limits<double>::infinity();
    isHit = false;
}


LocalGeo::LocalGeo(const LocalGeo &rhs) {
    point = rhs.point;
    normal = rhs.normal;
    tHit = rhs.tHit;
    isHit = rhs.isHit;
}


LocalGeo& LocalGeo::operator=(const LocalGeo &rhs) {
    if (this == &rhs) {
        return *this;
    }

    point = rhs.point;
    normal = rhs.normal;
    tHit = rhs.tHit;
    isHit = rhs.isHit;

    return *this;
}

#endif
