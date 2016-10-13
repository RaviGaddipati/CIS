//
// Created by gaddra on 10/12/16.
//

#ifndef CIS_CAL_POINTCLOUD_H
#define CIS_CAL_POINTCLOUD_H

#include "Eigen"
#include <vector>

template<typename T=int>
class PointCloud {
public:
    typedef Eigen::Matrix<T, 3, 1> Point;

    PointCloud(){}

    void add_point(const Point &point) {
        _cloud.push_back(point);
    }

    Point centroid() const {
        Point cent(0,0,0);
        for (const auto &p : _cloud) cent += p;
        cent /= _cloud.size();
        return cent;
    }

    PointCloud<T> operator+=(Point p) {
        for (auto &pnt : _cloud) pnt += p;
        return *this;
    }

    PointCloud<T> operator-=(Point p) {
        for (auto &pnt : _cloud) pnt -= p;
        return *this;
    }

    void center() {
        (*this) -= centroid();
    }

    size_t size() const {
        return _cloud.size();
    }

    typename std::vector<Point, Eigen::aligned_allocator<Point>>::const_iterator begin() const {
        return _cloud.begin();
    }

    typename std::vector<Point, Eigen::aligned_allocator<Point>>::const_iterator end() const {
        return _cloud.end();
    }

private:
    std::vector<Point, Eigen::aligned_allocator<Point>> _cloud;
};



#endif //CIS_CAL_POINTCLOUD_H
