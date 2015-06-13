#ifndef RAY_H
#define RAY_H

#include <boost/assert.hpp>
#include <Eigen/Dense>
#include <assert.h>
#include <cmath>
#include "consts.hpp"

/**
   A ray with position, direction, and magnitude.  

   @author Andrew Barbarello
 */
class Ray {
public:    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW /* Structures containing 
				       "fixed-sized vectorizable" Eigen classes
				       need proper memory alignment */
    Ray(const Eigen::Vector4d &orig, const Eigen::Vector4d &direction, const int fromObj=ID_AIR) :
	origin(orig), dir(direction), fromObj(fromObj) {
	BOOST_ASSERT_MSG(orig[3] == 1, 
			 "Origin of ray must have 4th coord equal to 1");
	BOOST_ASSERT_MSG(direction[3] == 0, 
			 "Direction of ray must have 4th coord equal to 0");
	BOOST_ASSERT_MSG(std::abs(direction.norm() - 1) < EPSILON, "Require unit direction vector!");
    }

     Eigen::Vector4d origin,/**< Origin of Ray in homogeneous coords */
	dir;/**< Direction of Ray in homogeneous coords */
	int fromObj; /**< ID of the object in the scene from which 
							this ray originated */
};

#endif
