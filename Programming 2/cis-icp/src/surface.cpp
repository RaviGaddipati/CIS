//
// Created by gaddra on 11/13/16.
//

#include <numeric>
#include "surface.h"

void cis::Surface::_reorder_longest_edge(Eigen::Vector3d &v1, Eigen::Vector3d &v2, Eigen::Vector3d &v3) {
    static double e1n,e2n,e3n;
    static Eigen::Vector3d swp;

    e1n = (v2 - v1).norm();
    e2n = (v3 - v1).norm();
    e3n = (v3 - v2).norm();

    if (e3n > e2n && e3n > e1n) {
        // v3-v2 is longest
        swp = v1;
        v1 = v3;
        v3 = swp;
    }

    else if (e2n > e1n && e2n > e3n) {
        // v2-v3 is longest
        swp = v2;
        v2 = v3;
        v3 = swp;
    }

    // Nothing to be done if v1-v2 is longest
}

Eigen::Vector3d cis::Surface::_bounding_sphere(const Eigen::Matrix<double, 9, Eigen::Dynamic> &triangle, double *radius) {
    static Eigen::Vector3d u,v,d, ret;
    static double g;
    static Eigen::Vector3d a,b,c;

    a = triangle.block<3,1>(0,0);
    b = triangle.block<3,1>(3,0);
    c = triangle.block<3,1>(6,0);

    _reorder_longest_edge(a, b, c);
    ret = (a + b).array() / 2;
    u = a - ret;
    v = c - ret;
    d = (u.cross(v)).cross(u);
    g = (v.norm() - u.norm()) / (d * 2).dot(v - u);

    if (g > 0 && std::isfinite(g)) ret += g * d;
    if (radius != nullptr) *radius = (ret - a).norm();
    return ret;
}

void cis::Surface::build() {
    assert(_neighbors.rows() == _triangles.cols());

    _spheres.resize(Eigen::NoChange, _triangles.cols());
    for(size_t i = 0; i < _spheres.cols(); ++i) {
        _spheres.col(i) = _bounding_sphere(_triangles.col(i));
    }
    std::vector<size_t> all(_spheres.cols());
    std::iota(all.begin(), all.end(), 0);
    _root = Division(all, this);
}

void cis::Surface::load(const Eigen::Array<double, 9, Eigen::Dynamic> &triangles,
                        const Eigen::Array<long, 3, Eigen::Dynamic> &neighbors) {
    _triangles = triangles;
    _neighbors = neighbors;
    build();
}

bool cis::Surface::Division::subdivide() {
    if (_left != nullptr && _right != nullptr) return true; // Already divided

    const int plane = _split_plane == 2 ? 0 : _split_plane + 1;
    const auto sphere_cmp = [this, plane](size_t a, size_t b)
    {
        return _surface->sphere_centroid(a)(plane) < _surface->sphere_centroid(b)(plane);
    };

    const auto middle = _included_spheres.begin() + _included_spheres.size()/2;
    std::nth_element(_included_spheres.begin(), middle, _included_spheres.end(), sphere_cmp);

    std::vector<size_t> left_n(_included_spheres.begin(), middle), right_n(middle, _included_spheres.end());
    if (left_n.size() == 0 || right_n.size() == 0) return false; // Further splits are useless
    _left = std::make_shared<Division>(left_n, _surface, plane);
    _right = std::make_shared<Division>(right_n, _surface, plane);

    return true;
}
