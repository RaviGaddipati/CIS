/**
 * @author Ravi Gaddipati
 * @date October 13, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * Represents a 3D Point Cloud as an Array of 3D Points.
 *
 * @file
 */

#ifndef CIS_CAL_POINTCLOUD_H
#define CIS_CAL_POINTCLOUD_H

#include "Eigen"
#include <vector>

/**
 * @brief
 * Represents a 3D point cloud.
 */
template<typename T=double>
class PointCloud {
public:
    typedef Eigen::Matrix<T, 3, 1> Point;
    typedef Eigen::Array<T, Eigen::Dynamic, 3> PointStore;

    PointCloud(){}

    /**
     * @brief
     * Create a point cloud from a vector of points
     * @param points
     */
    PointCloud(const std::vector<Point> &points) {
        add_points(points);
    }

    /**
     * @brief
     * Returns the i'th added point.
     * @param i
     * @return Point at i
     */
    Point at(size_t i) const {
        if (i >= _cloud_matrix.rows()) throw std::invalid_argument("Invalid index " + std::to_string(i));
        return _cloud_matrix.row(i);
    }

    /**
     * @brief
     * Refrence to a specific point
     * @param i index of point to retrieve
     * @return Point ref
     */
    void set(const size_t i, const Point &p) {
        if (i >= _cloud_matrix.rows()) throw std::range_error("Index out of range.");
        _cloud_matrix.row(i) = p;
    }

    /**
     * @brief
     * Add a point to the cloud.
     * @param point
     */
    void add_point(const Point &point) {
        _cloud_matrix.conservativeResize(_cloud_matrix.rows() + 1, Eigen::NoChange);
        _cloud_matrix.row(_cloud_matrix.rows() - 1) = point;
    }

    /**
     * @brief
     * Add a list of points of the cloud. More effecient than multiple add_points
     * as resize only happens once.
     * @param points
     */
    void add_points(const std::vector<Point> &points) {
        size_t r = _cloud_matrix.rows();
        _cloud_matrix.conservativeResize(_cloud_matrix.rows() + points.size(), Eigen::NoChange);
        for (const auto &p : points) {
            _cloud_matrix.row(r++) = p;
        }
    }

    /**
     * @brief
     * Returns the centroid.
     * P = sum(all_points) / num_points
     * @return Centroid Point
     */
    Point centroid() const {
        Point cent(_cloud_matrix.col(0).sum(), _cloud_matrix.col(1).sum(), _cloud_matrix.col(2).sum());
        return cent /= _cloud_matrix.rows();
    }

    /**
     * @brief
     * Add the given point to all points.
     * @param p Point to add
     * @return *this
     */
    PointCloud<T> operator+=(Point p) {
        _cloud_matrix.col(0) += p(0);
        _cloud_matrix.col(1) += p(1);
        _cloud_matrix.col(2) += p(2);
        return *this;
    }

    /**
     * @brief
     * Subtract the given point from all points.
     * @param p Point to subtract
     * @return *this
     */
    PointCloud<T> operator-=(Point p) {
        _cloud_matrix.col(0) -= p(0);
        _cloud_matrix.col(1) -= p(1);
        _cloud_matrix.col(2) -= p(2);
        return *this;
    }

    /**
     * @brief
     * Centers all the points around the centroid. Returns new PointCloud.
     * pc -= this->centroid()
     * @return Centered PointCloud
     */
    PointCloud<T> center() const {
        PointCloud<T> ret(*this);
        ret -= centroid();
        return ret;
    }

    /**
     * @brief
     * Centers all the points around the centroid. Modifies original PointCloud.
     * *this -= this->centroid()
     * @return *this
     */
    PointCloud<T> center_self() {
        *this -= centroid();
        return *this;
    }

    /**
     * @return number of points in cloud.
     */
    size_t size() const {
        return _cloud_matrix.rows();
    }

    /**
     * @brief
     * Get all the points.
     * @return Eigen::Array of points
     */
    const PointStore &point_store() const {
        return _cloud_matrix;
    }

    /**
     * @brief
     * Combine two point clouds into a single point cloud.
     * @param pc
     * @return PointCloud with all points of both sets.
     */
    PointCloud<T> operator+(const PointCloud<T> &pc) const {
        PointCloud<T> ret(*this);
        ret.add_points(pc.point_store());
        return ret;
    }

private:
    PointStore _cloud_matrix; // Stores all the points.
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
