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

int nChoosek(int n, int k) {

    if (k > n || k < 0) { return 0;} //If outside the bounds
    if (k == 0 || k == n) { return 1;}
    if (k*2 > n){k = n - k;} //Due to symmetry of combinations

    //Goal is to compute the simplified expression n(n-1)...(n-(k-1))/(1*2*...k) after removal of (n-k)! from numerator
    int res = n;
    for (int i = 2; i <= k; i++) {
        res = res * (n - (i - 1));
        res = res / i;
    }
    return res;
}