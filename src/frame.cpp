#include "lidro/isl100/frame.hpp"
#include <stdint.h>
#include <iostream>
#include <algorithm>
#include <cassert>
#include "log_message.h"


#define LOWORD(l)   ((unsigned short)((l) & 0xffff))
#define HIWORD(l)   ((unsigned short)(((l) >> 16) & 0xffff))

namespace lidro::isl100 {

Frame::Frame(stream_mode mode) 
: mode_(mode),
  slot_packing_factor_((mode_ == stream_mode_1 || mode_ == stream_mode_3) ? 4 : 8),
  sizeof_slot_((mode_ == stream_mode_1 || mode_ == stream_mode_2) ? 592 : 976),
  rflt_len_((mode_ == stream_mode_1 || mode_ == stream_mode_2) ? 96 * 4 : 192 * 4),
  width_((mode_ == stream_mode_1 || mode_ == stream_mode_3) ? 480 : 960),
  height_((mode_ == stream_mode_1 || mode_ == stream_mode_2) ? 96 : 192),
  d8_((mode_ == stream_mode_1 || mode_ == stream_mode_3) ? 0.250 : 0.125),
  max_distance_((mode_ == stream_mode_1 || mode_ == stream_mode_2) ? 30000 : 15000)
{

  if (mode_ == stream_mode_1) {
    max_intensity_ = 318;
  }
  else if (mode_ == stream_mode_2) {
    max_intensity_ = 141;
  }
  else if (mode_ == stream_mode_3) {
    max_intensity_ = 159;
  }
  else if (mode_ == stream_mode_4) {
    max_intensity_ = 70;
  }
  else {
    throw std::runtime_error("unknown stream mode");
  }

  ambient_data_ = std::make_unique<Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>>(ambi_height_, width_);
  distances_ = std::make_unique<Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic>>(height_, width_);
  intensities_ = std::make_unique<Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic>>(height_, width_);
}

size_t Frame::fetch_reflected_data(const uint8_t *buf, int frag_no, int slot_no) {
  auto read16 = [](uint8_t *data) -> uint16_t {
    return (uint16_t(data[0] & 0x00ff)) | uint16_t(0xFF00 & data[1] << 8);
  };

  auto read32 = [](uint8_t *data) -> uint16_t {
    return data[0] |
          (data[1] << 8) |
          (data[2] << 16) |
          (data[3] << 24);
  };

  const uint8_t *start = buf;
  const uint8_t *end = start + rflt_len_;
  const int col = frag_no * slot_packing_factor_ + slot_no;
  for(uint8_t *p = (uint8_t *)start; p < end; p+=4) {
    uint32_t value = read32(p);
    uint16_t distance = LOWORD(value);
    uint16_t intensity = HIWORD(value);

    if (intensity > max_intensity_) {
      LOG(WARNING) << "unexpected intensity: <" << intensity <<
        "(0x" << std::hex << intensity << ")"  << std::dec <<
        "> valid value range: [0, " <<  max_intensity_ << "]";
      // intensity = std::numeric_limits<uint16_t>::infinity();
      intensity = max_intensity_;
    }

    if (distance > max_distance_) {
      distance = std::numeric_limits<uint16_t>::infinity();
    }

    int row = (p-start)/4;
    
    (*distances_)(row, col) = distance;
    (*intensities_)(row, col) = intensity;
  }
  return rflt_len_;
}

size_t Frame::fetch_ambient_data(const uint8_t *buf, int frag_no, int slot_no) {
  const uint8_t *start = buf;
  const uint8_t *end = start + ambi_height_;
  const int col = frag_no * slot_packing_factor_ + slot_no;
  for(uint8_t *p = (uint8_t *)start; p < end; ++p){
    int row = p - start;
    (*ambient_data_)(row, col) = *p;
  }
  return ambi_height_;
}

size_t Frame::fetch(int frag_no, uint8_t *buf, size_t len) {  
  auto fetch_slot = [&](const uint8_t *buf, int frag_no, int slot_no){
    size_t offset =0;
    offset += 16; // reserved
    offset += fetch_ambient_data(&buf[offset], frag_no, slot_no);
    offset += fetch_reflected_data(&buf[offset], frag_no, slot_no);
    return offset;
  };

  size_t offset = 0;
  for (size_t slot_no = 0; slot_no < slot_packing_factor_; ++slot_no) {
    assert(offset == slot_no * sizeof_slot_);
    offset += fetch_slot(&buf[offset], frag_no, slot_no);
  }
  return offset;
}

} // end namespace lidro
