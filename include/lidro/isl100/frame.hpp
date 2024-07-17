#ifndef __LIDRO_ISL100_FRAME_H__
#define __LIDRO_ISL100_FRAME_H__

#include <cstdint>
#include <cstddef> // Include this for size_t
#include <vector>
#include <memory>
#include "lidro/isl100/config.hpp"
#include <Eigen/Dense>

namespace lidro::isl100 {

class Frame {
public:
  Frame(stream_mode mode);

  size_t fetch(int index, uint8_t *buf, size_t len);

  Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> &ambient_data() {
    return *ambient_data_;
  }

  Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic> &intensities() {
    return *intensities_;
  }

  Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic> &distances() {
    return *distances_;
  }

  size_t width() {return width_;}

  size_t height() {return height_;}

  size_t ambi_height() {return ambi_height_;}

  uint16_t max_distance() {return max_distance_;}

  uint16_t max_intensity() {return max_intensity_;}

  double d8() {return d8_;}

private:
  size_t fetch_reflected_data(const uint8_t *buf, int frag_no, int slot_no);
  size_t fetch_ambient_data(const uint8_t *buf, int frag_no, int slot_no);

private:
  std::unique_ptr<Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>> ambient_data_;
  std::unique_ptr<Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic>> intensities_;
  std::unique_ptr<Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic>> distances_;

  const stream_mode mode_;

  const int slot_packing_factor_;
  const int sizeof_slot_;
  const int rflt_len_;
  const int height_;
  const int ambi_height_{192};
  const int width_;
  const double d8_; // delta theta

  const uint16_t max_distance_;
  uint16_t max_intensity_;
};

} // end namespace lidro

#endif // __LIDRO_ISL100_FRAME_H__
