//
// Created by gaddra on 11/13/16.
//

#ifndef CIS_ICP_SURFACE_H
#define CIS_ICP_SURFACE_H

#include <memory>
#include "Eigen"
#include "doctest.h"

namespace cis {

    class Surface {
    public:
        class Division {
        public:
            Division() : _left(nullptr), _right(nullptr), _surface(nullptr), _split_plane(0) {};
            Division(const std::vector<size_t> &spheres,
                     const Surface * surface,
                     int plane=0) :
                    _included_spheres(spheres), _left(nullptr), _right(nullptr),
                    _surface(surface), _split_plane(plane) {}

            bool subdivide();

            size_t size() const {
                return _included_spheres.size();
            }

            Eigen::Array<double, 9, 1> at(size_t i) const {
                return _surface->triangle(i);
            };

            Division &left() {
                if (_left == nullptr) subdivide();
                return *_left;
            }

            Division &right() {
                if (_right == nullptr) subdivide();
                return *_right;
            }


        private:
            std::vector<size_t> _included_spheres; /**< Index of sphere in _spheres, _triangles to include */
            std::shared_ptr<Division> _left, _right;
            const Surface * _surface;
            int _split_plane;
        };

        Surface() = default;

        Surface(const Eigen::Array<double, 9, Eigen::Dynamic> &triangles,
                const Eigen::Array<long, 3, Eigen::Dynamic> &neighbors) :
                _triangles(triangles), _neighbors(neighbors) {
            build();
        }

        void build();

        void load(const Eigen::Array<double, 9, Eigen::Dynamic> &triangles,
                  const Eigen::Array<long, 3, Eigen::Dynamic> &neighbors) {
            _triangles = triangles;
            _neighbors = neighbors;
            build();
        }

        Eigen::Array<double, 9, 1> triangle(size_t i) const {
            return _triangles.col(i);
        }

        Eigen::Matrix<double, 3, Eigen::Dynamic> sphere_centroid(size_t i) const {
            return _spheres.col(i);
        };

    private:
        // Each col is centroid XYZ, followed by index of triangle in _triangles
        Eigen::Array<double, 3, Eigen::Dynamic> _spheres;
        // Each col is X,Y,Z of each vertex
        Eigen::Array<double, 9, Eigen::Dynamic> _triangles;
        Eigen::Array<long, 3, Eigen::Dynamic> _neighbors;
        Division _root;



        /**
         * @brief
         * Given 3 points, reorder such that the edge between v1 and v2 is the longest edge.
         * @param v1 Vertex 1
         * @param v2 Vertex 2
         * @param v3 Vertex 3
         */
        void _reorder_longest_edge(Eigen::Vector3d &v1, Eigen::Vector3d &v2, Eigen::Vector3d &v3);

        Eigen::Vector3d _bounding_sphere(const Eigen::Matrix<double, 9, Eigen::Dynamic> &triangle, double *radius=nullptr);

    };
}

TEST_CASE("Surface tree") {
;
}


#endif //CIS_ICP_SURFACE_H
