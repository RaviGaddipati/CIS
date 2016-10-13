//
// Created by gaddra on 10/12/16.
//

#ifndef CIS_CAL_POINTCLOUD_H
#define CIS_CAL_POINTCLOUD_H

#include "Eigen"
#include <vector>

template<typename T=double>
class PointCloud {
public:
    typedef Eigen::Matrix<T, 3, 1> Point;
    typedef Eigen::Array<T, Eigen::Dynamic, 3> PointStore;

    PointCloud(){}

    PointCloud(const std::vector<Point> &points) {
        add_points(points);
    }

    Point at(size_t i) const {
        if (i >= _cloud_matrix.rows()) throw std::invalid_argument("Invalid index " + std::to_string(i));
        return _cloud_matrix.row(i);
    }

    void add_point(const Point &point) {
        _cloud_matrix.conservativeResize(_cloud_matrix.rows() + 1, Eigen::NoChange);
        _cloud_matrix.row(_cloud_matrix.rows() - 1) = point;
    }

    void add_points(const std::vector<Point> &points) {
        size_t r = _cloud_matrix.rows();
        _cloud_matrix.conservativeResize(_cloud_matrix.rows() + points.size(), Eigen::NoChange);
        for (const auto &p : points) {
            _cloud_matrix.row(r++) = p;
        }
    }

    Point centroid() const {
        Point cent(_cloud_matrix.col(0).sum(), _cloud_matrix.col(1).sum(), _cloud_matrix.col(2).sum());
        return cent /= _cloud_matrix.rows();
    }

    PointCloud<T> operator+=(Point p) {
        _cloud_matrix.col(0) += p(0);
        _cloud_matrix.col(1) += p(1);
        _cloud_matrix.col(2) += p(2);
        return *this;
    }

    PointCloud<T> operator-=(Point p) {
        _cloud_matrix.col(0) -= p(0);
        _cloud_matrix.col(1) -= p(1);
        _cloud_matrix.col(2) -= p(2);
        return *this;
    }

    PointCloud<T> center() {
        PointCloud<T> ret(*this);
        ret -= centroid();
        return ret;
    }

    size_t size() const {
        return _cloud_matrix.rows();
    }

    const PointStore &point_store() const {
        return _cloud_matrix;
    }

private:
    PointStore _cloud_matrix;
};

template <typename T>
std::ostream &operator<<(std::ostream &os, const PointCloud<T> &pc) {
    os << pc.point_store();
    return os;
}

TEST_CASE ("Point Cloud") {

    std::vector<PointCloud<double>::Point> points = {{1,1,2}, {2,4,1}, {3,6,3}, {2,1,6}};
    PointCloud<double> pc(points);

            SUBCASE("Add points") {
        PointCloud<double>::Point p;

        p = {1,1,2};
                CHECK(pc.at(0) == p);
        p = {2,4,1};
                CHECK(pc.at(1) == p);
        p = {3,6,3};
                CHECK(pc.at(2) == p);
        p = {2,1,6};
                CHECK(pc.at(3) == p);
        CHECK_THROWS(pc.at(4));

        pc.add_point({2,2,2});
        p = {2,2,2};
        CHECK(pc.at(4) == p);
        CHECK(pc.size() == 5);
    }

    SUBCASE("Centroid") {
        auto cent = pc.centroid();
        PointCloud<double>::Point p{2, 3, 3};
        CHECK(cent == p);
    }

    SUBCASE("Center") {
        auto c = pc.center();
        PointCloud<double>::Point p;

        p = {-1,-2,-1};
                CHECK(c.at(0) == p);
        p = {0,1,-2};
                CHECK(c.at(1) == p);
        p = {1,3,0};
                CHECK(c.at(2) == p);
        p = {0,-2,3};
                CHECK(c.at(3) == p);

    }
}

#endif //CIS_CAL_POINTCLOUD_H
