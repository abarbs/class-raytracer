#ifndef BOX_H
#define BOX_H

#include <string>
#include <iostream>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include "QuadricCollection.hpp"

/**
   A class for representing a box in the scene. Expected to be constructed from 
   a string formatted as defined on the assigment page.

   @author Andrew Barbarello
 */

class Box : public QuadricCollection {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector4d center;
    Eigen::Vector4d u;
    Eigen::Vector4d v;
    Eigen::Vector4d w;
    
    Box(Eigen::Vector4d &center, Eigen::Vector4d& u, Eigen::Vector4d &v, 
	Eigen::Vector4d &w, unsigned int id, unsigned int matId, 
	double uWidth, double vWidth, double wWidth)
	: QuadricCollection(center, id, matId), u(u), v(v), w(w)  {
	BOOST_ASSERT_MSG(u[3] == 0 && v[3] == 0 && w[3] == 0, "u,v,w must have"
			 " fourth coordinate equal to zero!");//  Got u:\n"
			 // << u << "v:\n" << v << "w:\n" << w)
;
	// Prepare rotation matrix
	Eigen::Matrix4d R;
	R.row(0) = u;
	R.row(1) =  v;
	R.row(2) =  w;
	R.row(3) = Eigen::Vector4d(0, 0, 0, 1);
	
	/* Make all the planes forming the box (centered at origin). 
	   Plane normals will be unit vectors pointing in positive/negative
	   x, y, z directions. The points x on the plane with normal 
	   n = (a,b,c), distance d to origin satisfy n.p -d = 0, 
	   or x^T Q x = 0 where 
	   Q = |0   0   0   a/2|
	       |0   0   0   b/2|
	       |0   0   0   c/2|
	       |a/2 b/2 c/2  -d|
	  We define planes w.r.t. x, y, z axes, then rotate to u,v,w
	 */
	Eigen::Matrix4d posWPlane, negWPlane, posUPlane, negUPlane,
	    posVPlane, negVPlane;
	posWPlane = negWPlane = posUPlane = negUPlane = posVPlane =  negVPlane
	    = Eigen::Matrix4d::Zero();
	posUPlane.row(3) = posUPlane.col(3) = Eigen::Vector4d(0.5, 0, 0, -uWidth/2.0);
	negUPlane.row(3) = negUPlane.col(3) = Eigen::Vector4d(-0.5, 0, 0, -uWidth/2.0);

	posVPlane.row(3) = posVPlane.col(3) = Eigen::Vector4d(0, 0.5, 0, -vWidth/2.0);
	negVPlane.row(3) = negVPlane.col(3) = Eigen::Vector4d(0, -0.5, 0, -vWidth/2.0);

	posWPlane.row(3) = posWPlane.col(3) = Eigen::Vector4d(0, 0, 0.5, -wWidth/2.0);
	negWPlane.row(3) = negWPlane.col(3) = Eigen::Vector4d(0, 0, -0.5, -wWidth/2.0);

	addQuadric(R.transpose() * posWPlane * R);
	addQuadric(R.transpose() * negWPlane * R);
	addQuadric(R.transpose() * posUPlane * R);
	addQuadric(R.transpose() * negUPlane * R);
	addQuadric(R.transpose() * posVPlane * R);
	addQuadric(R.transpose() * negVPlane * R);
	
    }

    static Box fromString(const std::string &str) {
	// boost::regex boxReg("((\\r\\n|\\n)+)?obj box ID[\\t ]+\\d+[\\t ]+"
	// "mat ID \\d+[\\t ]+(([-+]?[0-9]*\\.?[0-9]+[\\t ]+){14})"
	// "([-+]?[0-9]*\\.?[0-9]+");
	// if (!boost::regex_match(str, boxReg))
	//     throw std::invalid_argument("Invalid box parameter string");
	
	std::stringstream paramStream(str);
	
	// Skip the "obj box ID" preamble
	std::string dummy;
	for (int i = 0; i < 3; ++i)
	    paramStream >> dummy;

	int id, matId;
	paramStream >> id;
	// Skip the "mat ID" preamble
	for (int i = 0; i < 2; ++i)
	    paramStream >> dummy;
	paramStream >> matId;
	Eigen::Vector4d center, u, v, w;
	center = Eigen::Vector4d::Ones();
	u = v = w = Eigen::Vector4d::Zero();
	for (int i = 0; i < 3; i++) {
	    paramStream >> center[i];
	}
	for (int i = 0; i < 3; i++) {
	    paramStream >> u[i];
	}
	for (int i = 0; i < 3; i++) {
	    paramStream >> v[i];
	}
	for (int i = 0; i < 3; i++) {
	    paramStream >> w[i];
	}
	
	double uWidth, vWidth, wWidth;
	paramStream >> uWidth >> vWidth >> wWidth;
	
	return Box(center, u, v, w, id, matId, uWidth, vWidth, wWidth);
    }

    
    
};

#endif
