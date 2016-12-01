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

#ifndef CIS_ICP_SURFACE_H
#define CIS_ICP_SURFACE_H

#include <memory>
#include <fstream>
#include <iostream>
#include "Eigen"
#include "pointcloud.h"

namespace cis {

    /**
     * @brief
     * Represents a surface as a mesh of triangles.
     */
    class Surface {
    public:
        /**
         * @brief
         * Allows for subdivisions of the surface represented as a kd-tree. Left and
         * right children are generated as they are requested.
         * @details
         * The node split point is always one of the points, no interpolation is done. Each split point
         * is not included in the children.
         */
        class Division {
        public:
            /**
             * @brief
             * Default constructor splits along the X axis.
             */
            Division() : _surface(nullptr), _split_plane(0) {};

            /**
             * @brief
             * Represent a surface division with
             * @param spheres bounding spheres included in the division
             * @param surface Core surface structure
             * @param plane The plane which to split children on.
             */
            Division(const std::vector<size_t> &spheres, const Surface * surface, int plane=0) :
            _included_spheres(spheres), _surface(surface), _split_plane(plane) {}

            /**
             * @brief
             * Computes the median of the included spheres, and splits them into
             * left and right children.
             */
            void subdivide();

            /**
             * @return the number of triangles in the current division
             */
            size_t size() const {
                return _included_spheres.size();
            }

            /**
             * @brief
             * Returns the 3 verticies of the triangle at index i
             * @param i Index of triangle
             * @return XYZ of each vertex in 9x1 matrix
             */
            Eigen::Array<double, 9, 1> at(size_t i) const {
                return _surface->triangle(i);
            };

            /**
             * @return ptr to left subtree
             */
            std::shared_ptr<Division> left() {
                if (_left == nullptr && _right == nullptr) subdivide();
                return _left;
            }

            /**
             * @return ptr to right subtree
             */
            std::shared_ptr<Division> right() {
                if (_left == nullptr && _right == nullptr) subdivide();
                return _right;
            }

            /**
             * @return The median of the included spheres
             */
            Point value() {
                if (_left == nullptr && _right == nullptr) subdivide();
                return _surface->_spheres.col(_middle);
            }

            /**
             * @return The axis along which the current node splits its children along.
             */
            int split_plane() const {
                return _split_plane;
            }

            /**
             * @brief
             * Given a point, find the closest point on the surface.
             * @param v
             * @return
             */
            Point find_closest_point(const Point &v);

            /**
             * @brief
             * Print the split point followed by all children, indentation determined by level.
             * @param level
             * @return multi-line string tree repersentation
             */
            std::string to_string(int level=0, bool ax=false);

        protected:
            std::vector<size_t> _included_spheres; /**< Index of sphere in _spheres, _triangles to include */
            size_t _middle; // Index of split point in _surface->_spheres
            std::shared_ptr<Division> _left = nullptr, _right = nullptr;
            const Surface * _surface;
            const int _split_plane;

            /**
             * @param v Find the closest point to v
             * @param bound distance from the closest point to v
             * @param closest candidate closest point
             */
            void _find_closest_impl(const Point &v, double &bound, Point &closest);
        };

        Surface() = default;

        /**
         * @brief
         * Construct a surface
         * @param triangles Array in which each col is XYZ of each vertex
         * @param neighbors Neighboring triangle indicies
         */
        Surface(const Eigen::Array<double, 9, Eigen::Dynamic> &triangles,
                const Eigen::Array<long, Eigen::Dynamic, 3> &neighbors) :
        _triangles(triangles), _neighbors(neighbors) {
            build();
        }

        /**
         * @brief
         * Builds the root kd-tree node and the bounding spheres of each triangle.
         */
        void build();

        /**
         * @brief
         * Construct a surface with the given triangles and neighbors.
         * @param triangles
         * @param neighbors
         */
        void load(const Eigen::Array<double, 9, Eigen::Dynamic> &triangles,
                  const Eigen::Array<long, 3, Eigen::Dynamic> &neighbors);

        /**
         * @return Root kd-tree node ptr
         */
        std::shared_ptr<Division> root() const {return _root;}

        /**
         * @brief
         * Return the triangle at a certain index.
         * @param i index
         * @return XYZ of each vertex of triangle
         */
        Eigen::Array<double, 9, 1> triangle(size_t i) const {
            return _triangles.col(i);
        }

        /**
         * @brief
         * Get the centroid of the bounding sphere of a triangle
         * @param i index of triangle
         * @return centroid of the triangles' bounding sphere
         */
        Eigen::Matrix<double, 3, Eigen::Dynamic> sphere_centroid(size_t i) const {
            return _spheres.col(i);
        };

        /**
         * @return The maximum radius of a triangles' bounding sphere in the surface.
         */
        inline double max_radius() const { return _max_radius; }

    private:
        // Each col is centroid XYZ, followed by index of triangle in _triangles
        Eigen::Array<double, 3, Eigen::Dynamic> _spheres;
        std::vector<double> _radii;
        // Each col is X,Y,Z of each vertex
        Eigen::Array<double, 9, Eigen::Dynamic> _triangles;
        Eigen::Array<long, Eigen::Dynamic, 3> _neighbors;
        std::shared_ptr<Division> _root;
        double _max_radius; // Maximum bounding sphere radius
        friend class Division;

        /**
         * @brief
         * Given 3 points, reorder such that the edge between v1 and v2 is the longest edge.
         * @param v1 Vertex 1
         * @param v2 Vertex 2
         * @param v3 Vertex 3
         */
        static void _reorder_longest_edge(Eigen::Vector3d &v1, Eigen::Vector3d &v2, Eigen::Vector3d &v3);

        /**
         * @brief
         * Compute the bounding sphere for a triangle
         * @param triangle
         * @param radius If the radius is desired, pass a double ptr
         * @return Sphere centroid
         */
        static Eigen::Vector3d
        _bounding_sphere(const Eigen::Matrix<double, 9, Eigen::Dynamic> &triangle, double *radius=nullptr);

    };

    /**
     * @brief
     * Check if a point should go to the left or right of the tree.
     * @param p Point we are searching for
     * @param d Surface to search
     * @return true if p is less than d in the split dimension of the node.
     */
    bool operator<=(const Point &p, Surface::Division &d);
    bool operator<=(const Point &p, std::shared_ptr<Surface::Division> d);

    /**
     * @brief
     * Print a tree.
     * @param os Output stream
     * @param d Root division to print all children of
     * @return Split point followed by children on a new line.
     */
    std::ostream &operator<<(std::ostream &os, Surface::Division &d);
    std::ostream &operator<<(std::ostream &os, std::shared_ptr<Surface::Division> d);

    /**
     * @brief
     * Project the point c onto the line segment with endpoints p,q.
     * @param c Point to project
     * @param p endpoint 1
     * @param q endpoint 2
     * @return Projected point
     */
    Point project_onto_segment(const Point &c, const Point &p, const Point &q);

    /**
     * @brief
     * Computes the closest point on the triangle, including the edges to point P.
     * @param p Projects this point onto triangle formed by verticies v1,v2,v3
     * @param v1 Vertex 1
     * @param v2 Vertex 2
     * @param v3 Vertex 3
     * @return Point on triangle closest to p
     */
    Point project_onto_triangle(const Point &p, const Point &v1, const Point &v2, const Point &v3);
}


#endif //CIS_ICP_SURFACE_H
