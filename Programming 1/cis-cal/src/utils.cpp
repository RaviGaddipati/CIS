/**
 * @author Ravi Gaddipati
 * @date November 23, 2015
 * rgaddip1@jhu.edu
 *
 * @brief
 * Contains common functions.
 *
 * @file
 */


#include <sstream>
#include <iostream>
#include "utils.h"


std::vector<std::string> split(const std::string &s, char delim) {
    /** Split string with delim, return a vector **/
    std::vector<std::string> newElems(0);
    split(s, delim, newElems);
    return newElems;
}


inline void split(const std::string &s, char delim, std::vector<std::string> &vec) {
    /** Split string with delim, return a vector **/
    std::istringstream ss(s);
    std::string item;
    vec.clear();

    if (s.length() == 0) {
        return;
    }
    else if (s.length() == 1 && s.at(0) != delim) {
        vec.push_back(s.substr(0, 1));
        return;
    }

    while (std::getline(ss, item, delim)) {
        if (item.size() != 0) vec.push_back(item);
    }

}


std::string getLastLine(std::ifstream& in) {
    std::string line, ret = "";
    while (in >> std::ws && std::getline(in, line)) {
        if (line.at(0) != '#') ret = line;
    }
    return ret;
}


std::string current_date() {
    time_t t = time(0);
    struct tm *now = localtime(&t);

    std::ostringstream ss;
    ss << (now->tm_year + 1900) << '-'
       << (now->tm_mon + 1) << '-'
       << now->tm_mday;
    return ss.str();
}

size_t nChoosek(size_t n, size_t k) {
    //If outside the bounds
    if (k > n) return 0;
    if (k == 0 || k == n) return 1;

    //Due to symmetry of combinations
    if (k*2 > n) {
        k = n - k;
    }

    //Goal is to compute the simplified expression n(n-1)...(n-(k-1))/(1*2*...k) after removal of (n-k)! from numerator
    size_t res = n;
    for (int i = 2; i <= k; i++) {
        res = res * (n - (i - 1));
        res = res / i;
    }
    return res;
}

const cis::Point min(const std::vector<cis::PointCloud> &cloud) {
    using namespace cis;
    Point min;
    {
        const PointStore &pcs = cloud.at(0).point_store();
        min(0) = pcs.col(0).minCoeff();
        min(1) = pcs.col(1).minCoeff();
        min(2) = pcs.col(2).minCoeff();
    }
    double v;
    for (const PointCloud &pc : cloud) {
        const PointStore &pcs = pc.point_store();
        v = pcs.col(0).minCoeff(); if (v < min(0)) min(0) = v;
        v = pcs.col(1).minCoeff(); if (v < min(1)) min(1) = v;
        v = pcs.col(2).minCoeff(); if (v < min(2)) min(2) = v;
    }
    return min;
}

const cis::Point max(const std::vector<cis::PointCloud> &cloud) {
    using namespace cis;
    Point max;
    {
        const PointStore &pcs = cloud.at(0).point_store();
        max(0) = pcs.col(0).maxCoeff();
        max(1) = pcs.col(1).maxCoeff();
        max(2) = pcs.col(2).maxCoeff();
    }
    double v;
    for (const PointCloud &pc : cloud) {
        const PointStore &pcs = pc.point_store();
        v = pcs.col(0).maxCoeff(); if (v < max(0)) max(0) = v;
        v = pcs.col(1).maxCoeff(); if (v < max(1)) max(1) = v;
        v = pcs.col(2).maxCoeff(); if (v < max(2)) max(2) = v;
    }
    return max;
}
