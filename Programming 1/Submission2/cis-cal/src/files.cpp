/**
 * @author Ravi Gaddipati, Doran Walsten
 * @date October 23, 2016
 * rgaddip1@jhu.edu
 *
 * @brief
 * File parser implementations.
 *
 * @file
 */
#include "files.h"

void cis::File::_parse_coordinates(std::istream &in, size_t n, cis::PointCloud &target) {
    std::string line;
    std::vector<std::string> split_line;
    std::vector<Point> points;
    for (size_t i = 0; i < n; ++i) {
        if (!std::getline(in, line)) throw std::invalid_argument("Unexpected end of file.");
        line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
        split(line, ',', split_line);
        if (split_line.size() != 3) throw std::invalid_argument("Expected 3 values in line: " + line);
        points.emplace_back(std::stod(split_line[0]), std::stod(split_line[1]), std::stod(split_line[2]));
    }
    target.add_points(points);
}

void cis::CalBody::open(std::istream &in) {
    // Parse meta info
    std::string line;
    std::getline(in, line);
    line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
    auto line_split = split(line, ',');

    size_t nd, na, nc;
    try {
        nd = std::stoul(line_split[0]);
        na = std::stoul(line_split[1]);
        nc = std::stoul(line_split[2]);
        this->_name = line_split[3];
    } catch (std::exception &e) {
        std::cerr << "Error parsing meta info line, expected 4 fields. \n" << line << std::endl;
        throw;
    }

    this->_clouds.resize(3);
    for (auto &c : this->_clouds) c.resize(1);
    this->_parse_coordinates(in, nd, this->_clouds[0][0]);
    this->_parse_coordinates(in, na, this->_clouds[1][0]);
    this->_parse_coordinates(in, nc, this->_clouds[2][0]);
}

void cis::OutputPraser::open(std::istream &in) {
    // Parse meta info
    std::string line;
    std::getline(in, line);
    line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
    auto line_split = split(line, ',');

    size_t nc, nf;
    try {
        nc = std::stoul(line_split[0]);
        nf = std::stoul(line_split[1]);
        this->_name = line_split[2];
    } catch (std::exception &e) {
        std::cerr << "Error parsing meta info line, expected 4 fields. \n" << line << std::endl;
        throw;
    }

    // EM Post coords
    std::getline(in, line);
    line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
    split(line, ',', line_split);
    assert(line_split.size() == 3);
    _em_post = {std::stod(line_split[0]), std::stod(line_split[1]), std::stod(line_split[2])};
    // Opt post
    std::getline(in, line);
    line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
    split(line, ',', line_split);
    assert(line_split.size() == 3);
    _opt_post = {std::stod(line_split[0]), std::stod(line_split[1]), std::stod(line_split[2])};


    this->_clouds.resize(2);
    for (auto &c : this->_clouds) c.resize(nf);
    for (size_t frame = 0; frame < nf; ++frame) {
        this->_parse_coordinates(in, nc, this->_clouds[0][frame]);
    }
}

void cis::OutputParser2::open(std::istream &in) {
    // Parse meta info
    std::string line;
    std::getline(in, line);
    line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
    auto line_split = split(line, ',');

    size_t nf;
    try {
        nf = std::stoul(line_split[0]);
        this->_name = line_split[1];
    } catch (std::exception &e) {
        std::cerr << "Error parsing meta info line, expected 2 fields. \n" << line << std::endl;
        throw;
    }

    _points.clear();
    for (size_t frame = 0; frame < nf; ++frame) {
        if(!std::getline(in, line)) {
            throw std::invalid_argument("Reached end of file, expected " +
                                        std::to_string(nf) + "frames, got " +
                                        std::to_string(frame));
        }
        line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
        split(line, ',', line_split);
        try {
            _points.emplace_back(std::stod(line_split[0]), std::stod(line_split[1]), std::stod(line_split[2]));
        } catch (std::exception &e) {
            std::cerr << "Error parsing line: " << line << std::endl;
            throw;
        }
    }
}

void cis::EMNav::open(std::istream &in) {
    // Parse meta info
    std::string line;
    std::getline(in, line);
    line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
    auto line_split = split(line, ',');

    size_t ng, nf;
    try {
        ng = std::stoul(line_split[0]);
        nf = std::stoul(line_split[1]);
        this->_name = line_split[2];
    } catch (std::exception &e) {
        std::cerr << "Error parsing meta info line, expected 3 fields. \n" << line << std::endl;
        throw;
    }

    this->_clouds.resize(1);
    for (auto &c : this->_clouds) c.resize(nf);
    for (size_t frame = 0; frame < nf; ++frame) {
        this->_parse_coordinates(in, ng, this->_clouds[0][frame]);
    }
}

void cis::EMFiducials::open(std::istream &in) {
    // Parse meta info
    std::string line;
    std::getline(in, line);
    line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
    auto line_split = split(line, ',');

    size_t ng, nf;
    try {
        ng = std::stoul(line_split[0]);
        nf = std::stoul(line_split[1]);
        this->_name = line_split[2];
    } catch (std::exception &e) {
        std::cerr << "Error parsing meta info line, expected 3 fields. \n" << line << std::endl;
        throw;
    }

    this->_clouds.resize(2);
    for (auto &c : this->_clouds) c.resize(nf);
    for (size_t frame = 0; frame < nf; ++frame) {
        this->_parse_coordinates(in, ng, this->_clouds[0][frame]);
    }
}

void cis::CTFiducials::open(std::istream &in) {
    // Parse meta info
    std::string line;
    std::getline(in, line);
    line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
    auto line_split = split(line, ',');

    size_t nb;
    try {
        nb = std::stoul(line_split[0]);
        this->_name = line_split[1];
    } catch (std::exception &e) {
        std::cerr << "Error parsing meta info line, expected 2 fields. \n" << line << std::endl;
        throw;
    }

    this->_clouds.resize(1);
    this->_clouds[0].resize(1);
    this->_parse_coordinates(in, nb, this->_clouds[0][0]);

}

void cis::OptPivot::open(std::istream &in) {
    // Parse meta info
    std::string line;
    std::getline(in, line);
    line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
    auto line_split = split(line, ',');

    size_t nd, nh, nf;
    try {
        nd = std::stoul(line_split[0]);
        nh = std::stoul(line_split[1]);
        nf = std::stoul(line_split[2]);
        this->_name = line_split[3];
    } catch (std::exception &e) {
        std::cerr << "Error parsing meta info line, expected 4 fields. \n" << line << std::endl;
        throw;
    }

    this->_clouds.resize(2);
    for (auto &c : this->_clouds) c.resize(nf);
    for (size_t frame = 0; frame < nf; ++frame) {
        this->_parse_coordinates(in, nd, this->_clouds[0][frame]);
        this->_parse_coordinates(in, nh, this->_clouds[1][frame]);
    }
}

void cis::CalReadings::open(std::istream &in) {
    // Parse meta info
    std::string line;
    std::getline(in, line);
    line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
    auto line_split = split(line, ',');

    size_t nd, na, nc, nf;
    try {
        nd = std::stoul(line_split[0]);
        na = std::stoul(line_split[1]);
        nc = std::stoul(line_split[2]);
        nf = std::stoul(line_split[3]);
        this->_name = line_split[4];
    } catch (std::exception &e) {
        std::cerr << "Error parsing meta info line, expected 5 fields. \n" << line << std::endl;
        throw;
    }

    this->_clouds.resize(3);
    for (auto &c : this->_clouds) c.resize(nf);
    for (size_t frame = 0; frame < nf; ++frame) {
        this->_parse_coordinates(in, nd, this->_clouds[0][frame]);
        this->_parse_coordinates(in, na, this->_clouds[1][frame]);
        this->_parse_coordinates(in, nc, this->_clouds[2][frame]);
    }
}

void cis::EMPivot::open(std::istream &in) {
    // Parse meta info
    std::string line;
    std::getline(in, line);
    line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
    auto line_split = split(line, ',');

    size_t ng, nf;
    try {
        ng = std::stoul(line_split[0]);
        nf = std::stoul(line_split[1]);
        this->_name = line_split[2];
    } catch (std::exception &e) {
        std::cerr << "Error parsing meta info line, expected 3 fields. \n" << line << std::endl;
        throw;
    }

    this->_clouds.resize(1);
    for (auto &c : this->_clouds) c.resize(nf);
    for (size_t frame = 0; frame < nf; ++frame) {
        this->_parse_coordinates(in, ng, this->_clouds[0][frame]);
    }
}

void cis::output_writer(std::ostream &out,
                          const std::string file_name,
                          const std::vector<cis::PointCloud> &data,
                          const Eigen::Matrix<double, 3, 1> &em_post,
                          const Eigen::Matrix<double, 3, 1> &opt_post) {
    if (data.size() < 1) throw std::invalid_argument("Invalid data vector.");

    out << data.at(0).size() << ',' << data.size() << ',' << file_name << '\n'
        << em_post(0) << ',' << em_post(1) << ',' << em_post(2) << '\n'
        << opt_post(0) << ',' << opt_post(1) << ',' << opt_post(2) << '\n';

    for (const auto &frame : data) {
        for (size_t i = 0; i < frame.size(); ++i) {
            out << frame.at(i)(0) << ',' << frame.at(i)(1) << ',' << frame.at(i)(2) << '\n';
        }
    }
    out << std::flush;
}

void cis::output_writer(const std::string file_name,
                          const std::vector<cis::PointCloud> &data,
                          const Eigen::Matrix<double, 3, 1> &em_post,
                          const Eigen::Matrix<double, 3, 1> &opt_post) {
    std::ofstream out(file_name);
    if (!out.good()) throw std::invalid_argument("Invalid output file: " + file_name);
    output_writer(out, file_name, data, em_post, opt_post);
}

void cis::output_writer(std::ostream &os, std::string name, const cis::PointCloud &frames) {
    os << frames.size() << ',' << name << '\n';
    for (size_t i = 0; i < frames.size(); ++i) {
        print_point(os, frames.at(i));
        os << '\n';
    }
}

void cis::output_writer(std::string filename, const cis::PointCloud &frames) {
    std::ofstream out(filename);
    if (!out.good()) throw std::invalid_argument("Error opening output file: " + filename);
    output_writer(out, filename, frames);
}
