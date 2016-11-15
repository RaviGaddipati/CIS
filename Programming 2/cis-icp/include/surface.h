//
// Created by gaddra on 11/13/16.
//

#ifndef CIS_ICP_SURFACE_H
#define CIS_ICP_SURFACE_H

#include <memory>
#include <fstream>
#include <iostream>
#include "Eigen"
#include "doctest.h"
#include "files.h"

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
                const Eigen::Array<long, Eigen::Dynamic, 3> &neighbors) :
                _triangles(triangles), _neighbors(neighbors) {
            build();
        }

        void build();

        void load(const Eigen::Array<double, 9, Eigen::Dynamic> &triangles,
                  const Eigen::Array<long, 3, Eigen::Dynamic> &neighbors);

        Division &root() {return _root;}

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
        Eigen::Array<long, Eigen::Dynamic, 3> _neighbors;
        Division _root;



        /**
         * @brief
         * Given 3 points, reorder such that the edge between v1 and v2 is the longest edge.
         * @param v1 Vertex 1
         * @param v2 Vertex 2
         * @param v3 Vertex 3
         */
        void _reorder_longest_edge(Eigen::Vector3d &v1, Eigen::Vector3d &v2, Eigen::Vector3d &v3);

        Eigen::Vector3d _bounding_sphere(const Eigen::Matrix<double, 9, Eigen::Dynamic> &triangle,
                                         double *radius=nullptr);

    };
}

TEST_CASE("Surface tree") {
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
    auto &root = s.root();

            SUBCASE("Centroids") {
        const std::vector<Eigen::Vector3d> pts = {{1,0.5,1}, {0.5,0.5,2}, {0.5,0,1}, {0.5,0.5,0}, {0.5,1,1},{0, 0.5,1}};
        for (size_t i = 0; i < root.size(); ++i) {
                    CHECK(s.sphere_centroid(i) == pts.at(i/2));
        }
    }

    SUBCASE("Tree") {
        CHECK(root.left().size() + root.right().size() == 12);
        //std::cerr << "\nLeft:\n";
        for (size_t i = 0; i < root.left().size(); ++i) {
            //std::cerr << root.left().at(i) << "\n\n";
        }

        //std::cerr << "\nRight:\n";
        for (size_t i = 0; i < root.right().size(); ++i) {
            //std::cerr << root.right().at(i) << "\n\n";
        }
    }



    remove(tmpfile.c_str());
}


#endif //CIS_ICP_SURFACE_H
