#ifndef CYLINDER_H
#define CYLINDER_H

#include <string>
#include <iostream>
#include <cmath>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include "QuadricCollection.hpp"

/**
   A class for representing a cylinder in the scene. Expected to be constructed from 
   a string formatted as defined on the assigment page.

   @author Andrew Barbarello
 */

class Cylinder : public QuadricCollection {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Cylinder(Eigen::Vector4d &center, unsigned int id, unsigned int matId, 
	     Eigen::Vector3d &n, /**< A unit vector in direction of cylinder's 
				   long axis */
	     double radius, double length): QuadricCollection(center, id, matId) {

	/* Find rotation taking z axis to n */
	const double dotProd = n.dot(Eigen::Vector3d::UnitZ());
	const Eigen::Vector3d rotAxis = (1 - abs(dotProd) < EPSILON) ? 
	    Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitZ().cross(n);
	const double theta = acos(dotProd);
	Eigen::Matrix3d rotMatrix;
	rotMatrix = Eigen::AngleAxisd(theta, rotAxis);
	
	Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
	R.topLeftCorner<3, 3>() = rotMatrix;

	/* Make the planes representing the end caps of the cylinder (defined
	   at the origin).
	   Plane normals will be unit vectors pointing in positive/negative
	   z directions. 
	*/
	Eigen::Matrix4d posPlane, negPlane;
	posPlane = negPlane = Eigen::Matrix4d::Zero();
	posPlane.row(3) = posPlane.col(3) = Eigen::Vector4d(0, 0, 0.5, -length/2.0);
	negPlane.row(3) = negPlane.col(3) = Eigen::Vector4d(0, 0, -0.5, -length/2.0);
      
	addQuadric(R * posPlane * R.transpose());
	addQuadric(R * negPlane * R.transpose());
	Eigen::Matrix4d Q;
	Q << 1/(radius * radius), 0, 0, 0,
	    0, 1/(radius * radius), 0, 0,
	    0, 0, 0, 0,
	    0, 0, 0, -1;
	
	addQuadric(R * Q  * R.transpose());
    }

    static Cylinder fromString(const std::string &str) {
	// boost::regex cylinderReg("obj cylinder ID[\\t ]+\\d+[\\t ]+"
	// "mat ID \\d+[\\t ]+(([-+]?[0-9]*\\.?[0-9]+[\\t ]+){7})"
	// "[-+]?[0-9]*\\.?[0-9]+");
	
	// if (!boost::regex_match(str, cylinderReg)) {
	//     throw std::invalid_argument("Invalid Cylinder parameter string");
	// }
	std::stringstream paramStream(str);
	
	// Skip the "obj cylinder ID" preamble
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
	Eigen::Vector3d n = Eigen::Vector3d::Zero();
	for (int i = 0; i < 3; i++) {
	    paramStream >> center[i];
	}
	for (int i = 0; i < 3; i++) {
	    paramStream >> n[i];
	}
	
	double radius, length;
	paramStream >> radius >> length;
	
	return Cylinder(center, id, matId, n, radius, length);
    }

    
    
};

#endif
