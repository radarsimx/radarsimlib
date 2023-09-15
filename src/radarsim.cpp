#include "radarsim.h"

#include <vector>

#include "transmitter.hpp"

struct c_Transmitter {
  void *ptr_transmitter;
};

c_Transmitter *Create_Transmitter(double *freq, double *freq_time,
                                  int waveform_size, double *freq_offset,
                                  double *pulse_start_time, int num_pulses,
                                  double *frame_start_time, int num_frames,
                                  float tx_power) {
  c_Transmitter *ptr_tx_c;
  Transmitter<float> *ptr_tx_cpp;

  ptr_tx_c = (c_Transmitter *)malloc(sizeof(*ptr_tx_c));

  std::vector<double> freq_vt;
  std::vector<double> freq_time_vt;
  freq_vt.reserve(waveform_size);
  freq_time_vt.reserve(waveform_size);
  for (int idx = 0; idx < waveform_size; idx++) {
    freq_vt.push_back(freq[idx]);
    freq_time_vt.push_back(freq_time[idx]);
  }

  std::vector<double> freq_offset_vt;
  std::vector<double> pulse_start_time_vt;
  freq_offset_vt.reserve(num_pulses);
  pulse_start_time_vt.reserve(num_pulses);
  for (int idx = 0; idx < num_pulses; idx++) {
    freq_offset_vt.push_back(freq_offset[idx]);
    pulse_start_time_vt.push_back(pulse_start_time[idx]);
  }

  std::vector<double> frame_start_time_vt;
  frame_start_time_vt.reserve(num_frames);
  for (int idx = 0; idx < num_frames; idx++) {
    frame_start_time_vt.push_back(frame_start_time[idx]);
  }

  ptr_tx_cpp =
      new Transmitter<float>(tx_power, freq_vt, freq_time_vt, freq_offset_vt,
                             pulse_start_time_vt, frame_start_time_vt);
  ptr_tx_c->ptr_transmitter = ptr_tx_cpp;

  return ptr_tx_c;
}

void Free_Transmitter(c_Transmitter *ptr_tx_c) {
  if (ptr_tx_c == NULL) return;
  delete static_cast<Transmitter<float> *>(ptr_tx_c->ptr_transmitter);
  free(ptr_tx_c);
}

// void mather_destroy(mather_t *m) {
//   if (m == NULL) return;
//   delete static_cast<CPPMather *>(m->obj);
//   free(m);
// }

// void mather_add(mather_t *m, int val) {
//   CPPMather *obj;

//   if (m == NULL) return;

//   obj = static_cast<CPPMather *>(m->obj);
//   obj->add(val);
// }

// int mather_val(mather_t *m) {
//   CPPMather *obj;

//   if (m == NULL) return 0;

//   obj = static_cast<CPPMather *>(m->obj);
//   return obj->val();
// }