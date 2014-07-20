#ifndef ELLIPSE_H
#define ELLIPSE_H

#include <iostream>
#include <string>
//#include <regex>
#include <boost/regex.hpp>
#include <Eigen/Dense>

/**
   A class for representing an ellipsoid in the scene. Expected to be constructed from 
   a string formatted as defined on the assigment page.



   @author Andrew Barbarello 
 */
class Ellipsoid : public QuadricCollection {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Ellipsoid(Eigen::Vector4d &center, unsigned int id, unsigned int matId, 
	double rx, double ry, double rz) : 
	QuadricCollection(center, id, matId) {
	Eigen::Matrix4d Q;
	Q << 1/(rx * rx), 0, 0, 0,
	    0, 1/(ry * ry), 0, 0,
	    0, 0, 1/(rz * rz), 0,
	    0, 0, 0, -1;
	addQuadric(Q);
    }

    static Ellipsoid fromString(const std::string &str) {
	boost::regex elRegex("((\\r\\n|\\n)+)?obj ellipsoid ID[\\t ]+\\d+"
				"[\\t ]+mat ID \\d+[\\t ]+"
				"(([-+]?[0-9]*\\.?[0-9]+[\\t ]*){6})"
				"((\\r\\n|\\n)+)?");
	
	if (!boost::regex_match(str, elRegex))
	    throw std::invalid_argument("Invalid box parameter string");
	
	std::stringstream paramStream(str);
	
	// Skip the "obj ellipsoid ID" preamble
	std::string dummy;
	for (int i = 0; i < 3; ++i)
	    paramStream >> dummy;

	int id, matId;
	paramStream >> id;
	// Skip the "mat ID" preamble
	for (int i = 0; i < 2; ++i)
	    paramStream >> dummy;
	paramStream >> matId;
	Eigen::Vector4d center = Eigen::Vector4d::Ones();
	for (int i = 0; i < 3; ++i) {
	    paramStream >> center[i];
	}
	double rx, ry, rz;
	paramStream >> rx >> ry >> rz;
	
	return Ellipsoid(center, id, matId, rx, ry, rz);

    }
};

#endif
