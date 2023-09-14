/*
 * This is an automatically generated code from:
 *
 *   {'model': './models/plate5x5.stl', 'location': (20, 0, 0), 'origin': (0, 0,
 * -2.5), 'speed': (-1, 0, 0), 'rotation': (-10, 0, 0), 'rotation_rate': (10, 0,
 * 0), 'permittivity': 2.0}
 */

#include "target_config.hpp"

void target_config(std::vector<Target<float>> &targets) {
  float points_t0[12] = {(float)0.0, (float)2.5, (float)-2.5, (float)0.0,
                         (float)2.5, (float)2.5, (float)0.0,  (float)-2.5,
                         (float)2.5, (float)0.0, (float)-2.5, (float)-2.5};
  int cells_t0[6] = {(int)0, (int)1, (int)2, (int)2, (int)3, (int)0};
  std::vector<zpv::Vec3<float>> loc_array_t0;
  std::vector<zpv::Vec3<float>> speed_array_t0;
  std::vector<zpv::Vec3<float>> rotation_array_t0;
  std::vector<zpv::Vec3<float>> rotrate_array_t0;

  loc_array_t0.push_back(zpv::Vec3<float>(20, 0, 0));
  speed_array_t0.push_back(zpv::Vec3<float>(-1, 0, 0));
  rotation_array_t0.push_back(zpv::Vec3<float>(-0.17453292519943295, 0.0, 0.0));
  rotrate_array_t0.push_back(zpv::Vec3<float>(0.17453292519943295, 0.0, 0.0));

  targets.push_back(Target<float>(
      points_t0, cells_t0, 2, zpv::Vec3<float>(0.0, 0.0, -2.5), loc_array_t0,
      speed_array_t0, rotation_array_t0, rotrate_array_t0,
      std::complex<float>((float)2.0, (float)0.0),
      std::complex<float>((float)1, (float)0), false));
}
