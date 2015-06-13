#ifndef MATERIAL_H
#define MATERIAL_H

#include <string>
#include <iostream>

#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>

/**
   A material of which objects in a scene may be composed.

   @author Andrew Barbarello
 */
class Material {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int id;
    Eigen::Vector4d rgb;
    double Ka, /**< Coefficient of ambient reflection */
	Kd, /**< Coefficient of diffuse reflection */
	Ks, /**< Coefficient of specular reflection */
	ns, /**< Specular reflection exponent */
	Kt, /**< Coefficient of transmisstion */
	Kr, /**< Coefficient of reflection */
	Irefract; /**< Index of refraction */
    /**
       Create a Material object from a string conforming to the
       model_file_format specification.
    */
    static Material fromString(const std::string str) {
	Material mat;
	std::stringstream istream(str);
	std::string objectType;
	istream >> objectType;
	if (!boost::iequals(objectType, "mat"))
	    throw std::runtime_error(boost::str(
	       boost::format("Called with invalid object description %1%") % str));
	std::string dummy;
	istream >> dummy; // Advance istream past "ID"
	
	istream >> mat.id >> mat.rgb[0] >> mat.rgb[1] >> mat.rgb[2] 
		>> mat.Ka >> mat.Kd >> mat.Ks >> mat.ns >> mat.Kt
		>> mat.Kr >> mat.Irefract;
	mat.rgb[3] = 0;
	return mat;

    }
};

inline std::ostream& operator<<(std::ostream& os, const Material& m) {
    const std::string formatStr("mat ID: %1%\n" 
				"rgb: %2%\nKa: %3% Kd: %4% Ks: %5% "
				"ns: %6% Kt: %7% Kr: %8% Irefract: %9%");
    os << boost::format(formatStr)  % m.id % m.rgb % m.Ka % m.Kd % m.Ks
	% m.ns % m.Kt % m.Kr % m.Irefract;
    return os;
}


#endif
