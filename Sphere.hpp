#ifndef SPHERE_H
#define SPHERE_H

#include <iostream>
#include <string>
//#include <regex>
#include <boost/regex.hpp>
#include <Eigen/Dense>

/**
   A class for representing a sphere in the scene. Expected to be constructed from 
   a string formatted as defined on the assigment page.



   @author Andrew Barbarello 
 */
class Sphere : public QuadricCollection {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Sphere(Eigen::Vector4d &center, unsigned int id, unsigned int matId, 
	   double radius) : QuadricCollection(center, id, matId) {
	Eigen::Matrix4d Q;
	double rSquared = radius * radius;
	Q << 1/rSquared, 0, 0, 0,
	    0, 1/rSquared, 0, 0,
	    0, 0, 1/rSquared, 0,
	    0, 0, 0, -1;
	addQuadric(Q);
    }

    static Sphere fromString(const std::string &str) {
	boost::regex sphereRegex("((\\r\\n|\\n)+)?obj sphere ID[\\t ]+\\d+"
				"[\\t ]+mat ID \\d+[\\t ]+"
				"(([-+]?[0-9]*\\.?[0-9]+[\\t ]*){4})"
				"((\\r\\n|\\n)+)?");
	
	if (!boost::regex_match(str, sphereRegex))
	    throw std::invalid_argument("Invalid box parameter string");
	
	std::stringstream paramStream(str);
	
	// Skip the "obj sphere ID" preamble
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
	double radius;
	paramStream >> radius;
	
	return Sphere(center, id, matId, radius);

    }
};

#endif
