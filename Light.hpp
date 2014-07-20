#ifndef LIGHT_H
#define LIGHT_H
#include <iostream>
#include <string>
#include <stdexcept>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <Eigen/Dense>

/**
	A light source in a scene.

   @author Andrew Barbarello
*/
class Light {
public:
    int id;
    enum LightType {POINT, INFINITE, AMBIENT };
#ifndef _UNIT_TESTS
private:    
#endif
    friend std::ostream& operator<<(std::ostream& os, const Light &l);
    LightType m_type;
    Eigen::Vector4d m_pos, m_dir;
    Eigen::Vector4d m_rgb;
    Eigen::Vector4d m_radialAttenuation;
    float m_angularAttenuation;
    bool m_shadowOn;
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW /* Structures containing 
				       "fixed-sized vectorizable" Eigen classes
				       need proper memory alignment */
    Light(int id, /**< Unique identifier of light source */
	  LightType type,
	  Eigen::Vector4d &pos, /**< Position of light source in 3d space */ 
	  Eigen::Vector4d &dir, /**< Light direction in 3d space */
	  Eigen::Vector4d &rgb, /**< RGB Light intensity */
	  /** Vector of coefficients used to compute radialAttenuation.
	      If d is distance to light, then light is attenuated by
	      a factor of 1/(radialAttenuation[0]*d^2 
	      + radialAttenuation[1] * d
	      + radialAttenuation[2]) */
	  Eigen::Vector4d &radialAttenuation, 
	  float angularAttenuation,
	  bool shadowOn
	  ) {
	this->id = id;
	m_type = type;
	m_pos = pos;
	m_dir = dir;
	m_rgb = rgb;
	m_radialAttenuation = radialAttenuation;
	m_angularAttenuation = angularAttenuation;
	m_shadowOn = shadowOn;
    }

    /**
       Create a Light object from a string conforming to the
       model_file_format specification.
     */
    static Light fromString(const std::string str) {
	std::stringstream istream(str);
	std::string objectType;
	istream >> objectType;
	if (!boost::iequals(objectType, "light"))
	    throw std::runtime_error(boost::str(
	      boost::format("Called with invalid object description %1%") % str));
	istream >> objectType; // Advance istream past "ID"

	// Parse rest of string
	int id; 
	LightType type;
	Eigen::Vector4d pos = Eigen::Vector4d::Ones(), dir = Eigen::Vector4d::Zero();
	Eigen::Vector4d rgb = Eigen::Vector4d::Zero();
	Eigen::Vector4d radialAttenuation = Eigen::Vector4d::Zero();
	float angularAttenuation;
	bool shadowOn;

	istream >> id;
	
	std::string lightType;
	istream >> lightType;
	if (boost::iequals(lightType, "pnt")) type = LightType::POINT;
        else if (boost::iequals(lightType, "inf")) type = LightType::INFINITE;
	else if (boost::iequals(lightType, "amb")) type = LightType::AMBIENT;
	else throw 
		 std::runtime_error(boost::str(
		     boost::format("Unrecognized light type %1%") % lightType));

	for (int i = 0; i < 3; ++i) {
	    istream >> pos[i];
	}
	    
	for (int i = 0; i < 3; ++i) {
	    istream >> dir[i];
	}
	dir.normalize();
	for (int i = 0; i < 3; ++i) {
	    istream >> rgb[i];
	}

	for (int i = 0; i < 3; ++i) {
	    istream >> radialAttenuation[i];
	}
	
	istream >> angularAttenuation;

	std::string shadowOption;
	istream >> shadowOption;
	if (boost::iequals(shadowOption, "shadow_on")) shadowOn = true;
	else if (boost::iequals(shadowOption, "shadow_off")) shadowOn = false;
	else throw std::runtime_error(boost::str(boost::format("Unrecognized shadow option %1% in input string %2%") % shadowOption % str));

	return Light(id, type, pos, dir, rgb, radialAttenuation,
		     angularAttenuation, shadowOn);
	
    }

    Eigen::Vector4d getPosition() const {return m_pos;}
    bool getShadowOn() const {return m_shadowOn;}
    bool isAmbient() const { return m_type == LightType::AMBIENT; }


    Eigen::Vector4d getVectorToLight(const Eigen::Vector4d &point) const {
	switch (m_type) {
	case LightType::POINT: {
	    return (m_pos - point).normalized();
	}
	case LightType::AMBIENT: {
	    throw std::runtime_error("Vector to ambient light is undefined");
	}
	case LightType::INFINITE: {
	    return -1 * m_dir;
	} 
        default:
            throw std::runtime_error("Unregonized light type");
	}
    }

    Eigen::Vector4d getAmountOfLight(const Eigen::Vector4d &point) const {
	Eigen::Vector4d intensity = Eigen::Vector4d::Zero();
	switch (m_type) {
	case LightType::POINT: {
		Eigen::Vector4d Vl = point - m_pos;
	    const double d = Vl.norm();
		double cosAngle = m_dir.dot(Vl.normalized());
		if (cosAngle < 0)
			return intensity;
		const double fl_angatten = pow(cosAngle, (double) m_angularAttenuation);
	    const double attenFactor = 1/(m_radialAttenuation[2]*d*d +
					  m_radialAttenuation[1]*d +
					  m_radialAttenuation[0]);
	    intensity =   fl_angatten * attenFactor * m_rgb;
	    break;
	}
	case LightType::AMBIENT:	
	case LightType::INFINITE:
	default: {
	    return m_rgb;
	    break;
	}
	}
	return intensity;
    }

    virtual ~Light() {}

};

inline std::ostream& operator<<(std::ostream& os, const Light& l) {
    const std::string formatStr("Light ID: %1% type: %2% pos:\n%3%\ndir: %4%\n" 
				"rgb: %5%\nradialAttenuation: %6%\n"
				"angularAttenuation: %7%");
    std::string lightType;
    if (l.m_type == Light::LightType::POINT) lightType = "pnt";
    else if (l.m_type == Light::LightType::INFINITE) lightType = "inf";
    else lightType = "amb";
    os << boost::format(formatStr)  % l.id % lightType % l.m_pos % l.m_dir 
	% l.m_rgb % l.m_radialAttenuation % l.m_angularAttenuation;
    return os;
}


#endif
