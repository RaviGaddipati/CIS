/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date Nov 16, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * Data file readers.
 *
 * @file
 */

#ifndef CIS_ICP_FILES_H
#define CIS_ICP_FILES_H

#include <string>
#include <fstream>
#include "pointcloud.h"
#include "surface.h"


namespace cis {

    /**
     * @brief
     * Abstract class that provides base data structure and coord parser.
     */
    class File {
    public:

        virtual void open(std::string file) {
            std::ifstream in(file);
            if (!in.good()) throw std::invalid_argument("Invalid file: " + file);
            open(in);
        }

        virtual void open(std::istream &) = 0;

        /**
         * @return Number of frames if data exists, else 0
         */
        virtual size_t size() const  {
            return _clouds.size() > 0 ? _clouds.at(0).size() : 0;
        };

        virtual std::string name() const {
            return _name;
        };

        virtual const std::vector<PointCloud> &get(const size_t i) const {
            return _clouds.at(i);
        }

    protected:
        std::string _name;
        std::vector<std::vector<PointCloud>>
                _clouds;

        /**
        * @brief
        * Parses the n next lines of the stream.
        * @param in input stream
        * @param n number of lines to parse
        * @param target Target to add points to.
        */
        char _parse_coordinates(std::istream &in, size_t n, PointCloud &target, const char delim);

    };

    /**
     * @brief
     * Represents a Calibration body.
     */
    class RigidBody : public File {
    public:
        using File::open;

        RigidBody() {}

        /**
         * @param file Open the given file
         */
        RigidBody(const std::string file) {
            this->open(file);
        }

        /**
         * @param in Parse data from the stream
         */
        void open(std::istream &in) override;

        /**
         * @return LED markers on the rigid body
         */
        const PointCloud &markers() const {
            return this->_clouds.at(0).at(0);
        }

        cis::Point tip() const {
            return _tip;
        }

    private:
        cis::Point _tip;

    };

    /**
     * Loads a file defining a surgace.
     */
    class SurfaceFile : public File {
    public:
        using File::open;

        SurfaceFile() {}
        SurfaceFile(const std::string file) {
            this->open(file);
        }

        void open(std::istream &in) override;

        /**
         * @return PointCloud of all the vertices.
         */
        const PointCloud &vertices() const {
            return this->_clouds.at(0).at(0);
        }

        /**
         * @return Array providing the indicies of each vertex for each triangle.
         */
        const Eigen::Array<long, Eigen::Dynamic, 3> &triangles() const {
            return this->_tri;
        }

        /**
         * @return All the triangles, where each column is XYZ of each vertex.
         */
        Eigen::Array<double, 9, Eigen::Dynamic> cat_triangles() const;

        /**
         * @return Indicies of neighboring triangles.
         */
        const Eigen::Array<long, Eigen::Dynamic, 3> &neighbor_triangles() const {
            return this->_neighbor;
        }

        /**
         * @return kd-tree of surface
         */
         Surface &surface() { return sur; }


    private:
        Eigen::Array<long, Eigen::Dynamic, 3> _tri, _neighbor;
        Surface sur;

    };

    class SampleReadings : public File {
    public:
        using File::open;

        SampleReadings() {}

        /**
         * @param file Open the given sample readings with N_a points in body A and N_b in body B
         */
        SampleReadings(std::string file, size_t N_a, size_t N_b) {
            this->N_a = N_a;
            this->N_b = N_b;
            open(file);
        }

        /**
         * @param in Parse the input stream
         */
        void open(std::istream &in);

        /**
         * @return Frames of LED markers on the pointer
         */
        const std::vector<PointCloud> &pointer_rigid_body() const {
            return this->_clouds[0];
        }

        /**
         * @return Frams of LED markers on the fixed rigid body in the bone
         */
        const std::vector<PointCloud> &fixed_rigid_body() const {
            return this->_clouds[1];
        }

    private:
        size_t N_a;
        size_t N_b;
    };
}


#endif //CIS_ICP_FILES_H