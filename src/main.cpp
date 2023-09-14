#include <iostream>
#include <vector>

#include "radar_config.hpp"
#include "scene.hpp"
#include "snapshot_config.hpp"
#include "target_config.hpp"

void run_scene_simulation() {
  std::cout << "start radar scene simulation" << std::endl;

  int level;
  float density = (float)0.001;
  std::vector<Target<float>> targets;
  target_config(targets);

  std::vector<Snapshot<float>> snapshots;
  level = snapshot_config(snapshots);

  Radar<float> radar;
  radar_config(radar);

  Scene<double, float> radar_scene;
  for (int idx = 0; idx < (int)targets.size(); idx++) {
    radar_scene.AddTarget(targets[idx]);
  }

  radar_scene.SetRadar(radar);

  int bb_size = (int)(radar.tx_.frame_size_ * radar.tx_.ptrh_channels_.size() *
                      radar.rx_.ptrh_channels_.size() * radar.tx_.pulse_size_ *
                      radar.rx_.sample_size_);
  double *bb_real = (double *)malloc(bb_size * sizeof(double));
  double *bb_imag = (double *)malloc(bb_size * sizeof(double));

  for (int idx = 0; idx < bb_size; idx++) {
    bb_real[idx] = 0.0;
    bb_imag[idx] = 0.0;
  }

  radar_scene.RunSimulator(level, false, snapshots, density, bb_real, bb_imag);

  free(bb_real);
  free(bb_imag);

  std::cout << "radar scene simulation completed" << std::endl;
}

int main() { run_scene_simulation(); }
