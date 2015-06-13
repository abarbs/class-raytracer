#ifndef RANDOMAASAMPLER_H
#define RANDOMAASAMPLER_H

#include "Camera.hpp"
#include "consts.hpp"

#define NUM_AA_GROUPS 6

class RandomAASampler {
  int aaGroupStart = 0;
  std::pair<double, double> AAoffsets[NUM_AA_GROUPS * NUM_AA_SUBPIXELS];

public:
  Camera c;

  RandomAASampler(Camera c) : c(c) {
    for (int i = 0; i < NUM_AA_GROUPS * NUM_AA_SUBPIXELS; ++i) {
      // Generate some ray offsets to loop through
      std::default_random_engine generator;
      std::uniform_real_distribution<double> uniformDist(0.0, 1.0);
      std::pair<double, double> offset;
      offset.first = uniformDist(generator);
      offset.second = uniformDist(generator);
      AAoffsets[i] = offset;
    }
  };

  void constructRaysThroughPixel(
      const int i, const int j,
      std::vector<Ray, Eigen::aligned_allocator<Ray>> &rays) {
    for (int subPx = 0; subPx < NUM_AA_SUBPIXELS; ++subPx) {
      double du, dv;
      du = (i - (c.viewportWidth / 2) + 1 +
            AAoffsets[aaGroupStart + subPx].first) *
           c.xRes;
      dv = (-j + (c.viewportHeight / 2) - 1 -
            AAoffsets[aaGroupStart + subPx].second) *
           c.yRes;

      Eigen::Vector4d copToImPlane = c.focalLength * c.n + du * c.u + dv * c.v;

      if (c.type == Camera::CameraType::ORTHOGRAPHIC) {
        rays[subPx] = Ray(c.cop + copToImPlane, c.n, ID_AIR);
      } else {
        // type == CameraType::PERSPECTIVE
        rays[subPx] =
            Ray(c.cop + copToImPlane, copToImPlane.normalized(), ID_AIR);
      }
    }
    aaGroupStart = (aaGroupStart + NUM_AA_SUBPIXELS) % NUM_AA_GROUPS;
  }
};

#endif /* RANDOMAASAMPLER_H */
