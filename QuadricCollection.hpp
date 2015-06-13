#ifndef QUADRIC_COLL_H
#define QUADRIC_COLL_H

#include <vector>
#include <assert.h>
#include <limits>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "SceneObject.hpp"

/**
   An object composed of a collection of quadric surfaces. Quadrics
   are given as the 4x4 matrix description Q such that all points x on
   the surface statisfy x'Qx == 0. Q should describe and origin centered
   quadric, which will be translated by the center point provided at creation
   of object.



   @author Andrew Barbarello
*/
class QuadricCollection : public SceneObject {

#ifdef TEST_QC_DIRECTLY
public:
#else
protected:
#endif
    QuadricCollection(const Eigen::Vector4d& center, int id,
		      int matId) : center(center) {
	this->id = id;
	this->matId = matId;
	BOOST_ASSERT_MSG(center[3] == 1,
			 "Center must have 4th coord equal to 1");
    }
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > quadrics;

    virtual void addQuadric(const Eigen::Matrix4d& quadric) {
	BOOST_ASSERT_MSG(quadric.isApprox(quadric.transpose()),
			 "Quadric must be a symmetric matrix");
	Eigen::Matrix4d Tin = Eigen::Matrix4d::Identity();
	Tin.block<3, 1>(0, 3) = -1 * this->center.head<3>();
	this->quadrics.push_back(Tin.transpose() * quadric * Tin);
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector4d center;
    bool onSurface(const Eigen::Vector4d &pt) {
	bool onBoundary = false;
	double distToSurface;
        // pt needs to be on or inside all quadrics to qualify as "on surface"
	for (const auto& Q : quadrics) {
                if ((distToSurface = pt.transpose() * Q * pt) > EPSILON)
                    return false; // pt not in intersection of all quadrics
                else if (std::abs(distToSurface) < EPSILON)
                    onBoundary = true; // pt on surface of at least _one_ of the quadrics
	}
	return onBoundary;
    }

    virtual double distToIntersection(const Ray &ray) {
	double dist = std::numeric_limits<double>::infinity();
	BOOST_ASSERT_MSG(std::abs(ray.dir.norm() - 1) < EPSILON, "Require unit direction vector!");
	const Eigen::Vector4d &p0 = ray.origin,
	    &u = ray.dir;
	for (unsigned int i = 0; i < quadrics.size(); ++i) {
		Eigen::Matrix4d &Q = quadrics[i];
	    const double a = u.transpose() * Q * u,
		b = 2 * u.transpose() * Q * p0,
		c = p0.transpose() * Q * p0;
	    const double discriminant = b * b - 4 * a * c;
	    if (discriminant < 0) {
		continue;
	    }
	    if (a == 0 && b != 0) {
		double tmp = -c/b;
		dist = (tmp > EPSILON && tmp < dist && onSurface(p0 + tmp*u)) ? tmp : dist;
		continue;
	    }
	    const double rootOfDiscriminant = sqrt(discriminant);
	    const double tminus = (-b - rootOfDiscriminant)/(2*a),
		tplus = (-b + rootOfDiscriminant)/(2*a);
	    if (tminus < 0) {
		if (tplus < 0) {
		    continue;
		} else {
		    dist = (tplus < dist && onSurface(p0 + tplus*u)) ? tplus : dist;
		    continue;
		}
	    } else if (c <= EPSILON && tplus < dist && onSurface(p0 + tplus*u)) {
		/* If ray originated on quadric, intersection should be
		   with exit point p0 + tplus * u,
		   else with exit point (for planes intersections,
		   tplus == tminus == 0) */
		dist = tplus;
	    } else if (onSurface(p0 + tminus*u)) {
		dist = (tminus < dist) ? tminus : dist;
	    }
	}
	return dist;
    }
    virtual Eigen::Vector4d getUnitNormal(const Eigen::Vector4d &point) {
	BOOST_ASSERT_MSG(point[3] == 1, "getNormal needs a homogeneous point!");
	BOOST_ASSERT_MSG(onSurface(point), "Asked for normal at point not on surface!");
	for (unsigned int i = 0; i < quadrics.size(); ++i) {
		Eigen::Matrix4d &Q = quadrics[i];
	    double val = point.transpose() * Q * point;
	    if (std::abs(val) < EPSILON) {
		return (2 * Q * point).cwiseProduct(Eigen::Vector4d(1, 1, 1, 0)).normalized();
	    }
	}

	BOOST_ASSERT_MSG(false, "Reached point we didn't think was possible!");
	/* Should never reach this point */
	return Eigen::Vector4d::Zero();
    }

    //virtual ~QuadricCollection() {}

};

#endif
