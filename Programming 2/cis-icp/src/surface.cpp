/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date Nov 30, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * Represents a surface as a Kd-tree
 *
 * @file
 */

#include <numeric>
#include <random>
#include "surface.h"
#include "doctest.h"
#include "utils.h"


void cis::Surface::_reorder_longest_edge(Eigen::Vector3d &v1, Eigen::Vector3d &v2, Eigen::Vector3d &v3) {
    Eigen::Vector3d swp;

    double e1n = (v2 - v1).squaredNorm();
    double e2n = (v3 - v1).squaredNorm();
    double e3n = (v3 - v2).squaredNorm();

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

Eigen::Vector3d
cis::Surface::_bounding_sphere(const Eigen::Matrix<double, 9,Eigen::Dynamic> &triangle, double *radius) {

    Eigen::Vector3d u,v,d, ret;
    Eigen::Vector3d a,b,c;
    double g;

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

    _max_radius = 0;

    _spheres.resize(Eigen::NoChange, _triangles.cols());
    _radii.resize(_triangles.cols());
    for(size_t i = 0; i < _spheres.cols(); ++i) {
        _spheres.col(i) = _bounding_sphere(_triangles.col(i), &(_radii[i]));
        if (_radii[i] > _max_radius) _max_radius = _radii[i];
    }
    std::vector<size_t> all(_spheres.cols());
    std::iota(all.begin(), all.end(), 0);
    _root = std::make_shared<Division>(all ,this);
}

void cis::Surface::load(const Eigen::Array<double, 9, Eigen::Dynamic> &triangles,
                        const Eigen::Array<long, 3, Eigen::Dynamic> &neighbors) {
    _triangles = triangles;
    _neighbors = neighbors;
    build();
}

void cis::Surface::Division::subdivide() {
    if (_left != nullptr || _right != nullptr) return; // Already divided
    if (_included_spheres.size() == 1) {
        // This node is a leaf node
        _middle = _included_spheres[0];
        return;
    }

    const int plane = (_split_plane + 1) % 3;
    const auto middle = _included_spheres.begin() + _included_spheres.size()/2;
    std::nth_element(_included_spheres.begin(), middle, _included_spheres.end(), [this, plane](size_t a, size_t b)
    {
        return _surface->sphere_centroid(a)(plane) < _surface->sphere_centroid(b)(plane);
    });

    _middle = *middle;

    if (middle - _included_spheres.begin() > 0) {
        _left = std::make_shared<Division>(std::vector<size_t>(_included_spheres.begin(), middle), _surface, plane);
    }
    else {
        _left = nullptr;
    }
    if (_included_spheres.end() - middle - 1 > 0) {
        _right = std::make_shared<Division>(std::vector<size_t>(middle + 1, _included_spheres.end()), _surface, plane);
    }
    else {
        _right = nullptr;
    }
}

void cis::Surface::Division::_find_closest_impl(const cis::Point &v, double &bound, cis::Point &closest) {
    subdivide();
    const Eigen::Vector3d scenter = _surface->_spheres.col(_middle);

    double dist = (v - scenter).norm() - _surface->_radii.at(_middle);
    if (dist < bound) {
        const auto &tri = _surface->_triangles.col(_middle);
        const auto cp = project_onto_triangle(v, tri.block<3,1>(0,0), tri.block<3,1>(3,0), tri.block<3,1>(6,0));
        dist = (cp - v).norm();
        if (dist < bound) {
            bound = dist;
            closest = cp;
        }
    }


    if (v < *this) {
        if (_left) {
            _left->_find_closest_impl(v, bound, closest);
        }
        if (_right && v(_split_plane) - scenter(_split_plane) - _surface->_max_radius < bound) {
            _right->_find_closest_impl(v, bound, closest);
        }
    } else {
        if (_right) {
            right()->_find_closest_impl(v, bound, closest);
        }
        if (_left && v(_split_plane) - scenter(_split_plane) - _surface->_max_radius < bound) {
            _left->_find_closest_impl(v, bound, closest);
        }
    }

}

std::string cis::Surface::Division::to_string(int level) {
    std::string ret = "";
    for (int i = 0; i < level; ++i) {
        ret += '\t';
    }
    ret += std::to_string(value()(0)) + "," +
           std::to_string(value()(1)) + "," +
           std::to_string(value()(2)) + "\n";
    if (_left) ret += _left->to_string(level + 1);
    if (_right) ret += _right->to_string(level + 1);
    return ret;
}

cis::Point cis::Surface::Division::find_closest_point(const cis::Point &v) {
    Point ret;
    double bnd = std::numeric_limits<double>::max();
    _find_closest_impl(v, bnd, ret);
    return ret;
}

bool cis::operator<(const cis::Point &p, cis::Surface::Division &d) {
    return p(d.split_plane()) < d.value()(d.split_plane());
}

bool cis::operator<(const cis::Point &p, std::shared_ptr<cis::Surface::Division> d) {
    return p < *d;
}

cis::Point cis::project_onto_segment(const cis::Point &c, const cis::Point &p, const cis::Point &q) {
    const Point qmp = q - p;
    return p + ( max(0.0, min((c-p).dot(qmp) / qmp.dot(qmp), 1.0)) * qmp );
}

cis::Point cis::project_onto_triangle(const cis::Point &p,
                                      const cis::Point &v1, const cis::Point &v2, const cis::Point &v3) {
    // Define triangle as vertex and two edges
    const Eigen::Vector3d u = v2 - v1,
    v = v3 - v1,
    w = p - v1,
    n = u.cross(v);
    const double inv_n_sq = 1.0 / n.dot(n);

    // Barycentric coords
    const double b2 = (u.cross(w)).dot(n) * inv_n_sq,
    b1 = (w.cross(v)).dot(n) * inv_n_sq,
    b0 = 1 - b1 - b2;

    // If all barycentric coords are >= 0, inside triangle / on edge
    if (b2 >= 0 && b1 >= 0 && b0 >= 0) return Point((v1 * b0) + (v2 * b1) + (v3 * b2));

    // Lies on opposite side of v1
    if (b0 <= 0) {
        if (b2 <= 0) return v2;
        if (b1 <= 0) return v3;
        return project_onto_segment(p, v2, v3);
    }

    // Lies on opposite side of v2
    if (b1 <= 0) {
        if (b2 <= 0) return v1;
        return project_onto_segment(p, v1, v3);
    }

    // Lies on opposite side of v3
    assert(b2 <= 0 && "Default case should always be true if previous projections are not");
    return project_onto_segment(p, v1, v2);
}

std::ostream &cis::operator<<(std::ostream &os, cis::Surface::Division &d) {
    os << d.to_string();
    return os;
}

std::ostream &cis::operator<<(std::ostream &os, std::shared_ptr<cis::Surface::Division> d) {
    return os << *d;
}

TEST_CASE("Surface tree") {
    /**
     * The test case defines a cube, where each face is split into two triangles. This allows for the
     * testing of bounding sphere centroids that lie on the median split plane.
     */
    const std::string tmpfile = "cis-icp-doctest.tmp";
    {
        std::ofstream o(tmpfile);
        o << "8\n"
          << "0 0 0\n"
          << "1 1 0\n"
          << "0 1 0\n"
          << "0 0 2\n"
          << "0 1 2\n"
          << "1 1 2\n"
          << "1 0 2\n"
          << "1 0 0\n"
          << "12\n"
          << "7 5 6 -1 -1 -1\n"
          << "7 1 5 -1 -1 -1\n"
          << "4 3 6 -1 -1 -1\n"
          << "4 5 6 -1 -1 -1\n"
          << "0 3 6 -1 -1 -1\n"
          << "7 0 6 -1 -1 -1\n"
          << "7 0 2 -1 -1 -1\n"
          << "7 2 1 -1 -1 -1\n"
          << "1 2 5 -1 -1 -1\n"
          << "5 2 4 -1 -1 -1\n"
          << "0 2 4 -1 -1 -1\n"
          << "0 4 3 -1 -1 -1\n";
    }

    cis::SurfaceFile f(tmpfile);
    cis::Surface s(f.cat_triangles(), f.neighbor_triangles());

    SUBCASE("Centroids") {
        const std::vector<Eigen::Vector3d> pts = {{1,0.5,1}, {0.5,0.5,2}, {0.5,0,1}, {0.5,0.5,0}, {0.5,1,1},{0, 0.5,1}};
        for (size_t i = 0; i < s.root()->size(); ++i) {
            CHECK(s.sphere_centroid(i) == pts.at(i/2));
        }

    }

    SUBCASE("Tree node sizes") {
        std::vector<std::shared_ptr<cis::Surface::Division>> q = { s.root() };
        while (q.size()) {
            std::shared_ptr<cis::Surface::Division> curr = q.back();
            q.pop_back();
            if (curr->size() > 1) {
                int size = 0;
                if (curr->left()) {
                    size += curr->left()->size();
                    q.push_back(curr->left());
                }
                if (curr->right()) {
                    size += curr->right()->size();
                    q.push_back(curr->right());
                }
                CHECK(size == curr->size() - 1);
            }
        }
    }

    SUBCASE("Closest Point") {
        //Generate random points with x >= 1, 1 >= y > 0, 2 >= z >= 0 (project onto face of prism)
        std::default_random_engine generator(rand() % 10);

        std::uniform_real_distribution<double> range_distribution(1.0,20.0);
        std::uniform_real_distribution<double> step_distribution(0.0,1.0);
        std::uniform_real_distribution<double> z_distribution(0.0,2.0);
        cis::PointCloud pc1;

        for (int j = 0; j < 5; j++) {
            cis::Point toAdd = {range_distribution(generator), step_distribution(generator), z_distribution(generator)};
            pc1.add_point(toAdd);
        }

        //Generate random points with 1 >= x >= 0 , y >= 1, 2 >= z >= 0 (project onto another face of prism
        cis::PointCloud pc2;
        for (int j = 0; j < 5; j++) {
            cis::Point toAdd = {step_distribution(generator), range_distribution(generator), z_distribution(generator)};
            pc2.add_point(toAdd);
        }

        //Generate random point x >=1, y >=1, 2 >= z >= 0 (project onto an edge of the prism
        cis::PointCloud pc3;
        for (int j = 0; j < 5; j++) {
            cis::Point toAdd = {range_distribution(generator), range_distribution(generator), z_distribution(generator)};
            pc3.add_point(toAdd);
        }

        //For each of the test point clouds, check to confirm that the expected closest point is determined
        //pc1 : in the form (1,y,z)
        for (size_t i = 0; i < pc1.size(); i++ ) {
            const auto c = s.root()->find_closest_point(pc1.at(i));
            CHECK(c(0) == 1);
        }

        //pc2: in the form (x,1,z)
        for (size_t i = 0; i < pc2.size(); i++ ) {
            const auto c = s.root()->find_closest_point(pc2.at(i));
            CHECK(c(1) == 1);
        }

        //pc3: in the form (1,1,z)
        for (size_t i = 0; i < pc3.size(); i++ ) {
            const auto c = s.root()->find_closest_point(pc3.at(i));
            CHECK(c(0) == 1);
            CHECK(c(1) == 1);
        }


    }


    remove(tmpfile.c_str());
}