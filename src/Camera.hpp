#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <string>
#include <random>
//#include <regex>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <Eigen/Dense>

#include "Ray.hpp"
#include "consts.hpp"

/**

The camera defining the render.

@author Andrew Barbarello
*/
class Camera {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum CameraType { PERSPECTIVE, ORTHOGRAPHIC };
  enum CameraType type;
  double xRes, yRes;
  int viewportWidth, viewportHeight;
  Eigen::Vector3d look, up;
  Eigen::Vector4d cop, u, v, n;
  double focalLength, nearClipDist, farClipDist;

  Ray constructRayThroughPixel(const int i, const int j) const {
    double du, dv;
    du = (i - (viewportWidth / 2) + 1) * xRes;
    dv = (-j + (viewportHeight / 2) - 1) * yRes;

    Eigen::Vector4d copToImPlane = focalLength * n + du * u + dv * v;

    if (type == CameraType::ORTHOGRAPHIC) {
      return Ray(cop + copToImPlane, n, ID_AIR);
    } else {
      // type == CameraType::PERSPECTIVE
      return Ray(cop + copToImPlane, copToImPlane.normalized(), ID_AIR);
    }
  }

  static Camera fromString(const std::string &str) {
    boost::regex camReg("(?:camera)[\\t ]+(perspective|orthographic)"
                        "[\\t ]+(([-+]?[0-9]*\\.?[0-9]+[\\t ]*){12})");
    boost::regex vportReg("viewport[\\t ]+(\\d+)[\\t ]+(\\d+)");
    boost::regex resReg("resolution[\\t ]+([-+]?[0-9]*\\.?[0-9]+)[\\t ]+"
                        "([-+]?[0-9]*\\.?[0-9]+)");

    boost::smatch camMatches, vportMatches, resMatches;

    if (!(boost::regex_search(str, camMatches, camReg) &&
          boost::regex_search(str, vportMatches, vportReg) &&
          boost::regex_search(str, resMatches, resReg))) {
      throw std::invalid_argument("Invalid camera parameter string");
    }

    std::string cameraTypeString = camMatches[1];
    CameraType type = boost::iequals(cameraTypeString, "perspective")
                          ? CameraType::PERSPECTIVE
                          : CameraType::ORTHOGRAPHIC;

    Eigen::Vector3d cop3d, look, up;
    std::stringstream paramStream(camMatches[2]);
    // Read in COP params
    for (int i = 0; i < 3; i++) {
      paramStream >> cop3d[i];
    }
    Eigen::Vector4d cop;
    cop << cop3d, 1;
    // Read in look at vector params
    for (int i = 0; i < 3; i++) {
      paramStream >> look[i];
    }
    // Read in look up vector params
    for (int i = 0; i < 3; i++) {
      paramStream >> up[i];
    }
    // Compute u, v, n according to Hearn, Baker, Carithers	4th ed
    // pg 313: n is the "viewplane" normal - a unit vector
    // in the direction -from- the look-at point, -to- the view plane,
    // (u, v) are (x, y) of the viewplane
    const Eigen::Vector3d n3d = (cop3d - look).normalized(),
                          u3d = (up.cross(n3d)).normalized(),
                          v3d = n3d.cross(u3d);
    Eigen::Vector4d u, v, n;
    u << u3d, 0;
    v << v3d, 0;
    n << n3d, 0;
    n = -1 * n;
    double focalLength, nearClipDist, farClipDist;
    paramStream >> focalLength >> nearClipDist >> farClipDist;
    int viewportWidth = boost::lexical_cast<int>(vportMatches[1]);
    int viewportHeight = boost::lexical_cast<int>(vportMatches[2]);
    double xRes = boost::lexical_cast<double>(resMatches[1]);
    double yRes = boost::lexical_cast<double>(resMatches[2]);

    Camera c;
    c.type = type;
    c.xRes = xRes;
    c.yRes = yRes;
    c.viewportWidth = viewportWidth;
    c.viewportHeight = viewportHeight;
    c.look = look;
    c.up = up;
    c.cop = cop;
    c.u = u;
    c.v = v;
    c.n = n;
    c.focalLength = focalLength;
    c.nearClipDist = nearClipDist;
    c.farClipDist = farClipDist;

    return c;
    // return  { type, xRes, yRes, viewportWidth, viewportHeight, look,
    // up, cop, u, v, n, focalLength, nearClipDist, farClipDist };
  }
};

#endif
