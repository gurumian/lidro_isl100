#ifndef LIDRO_ISL100_CONFIG_H_
#define LIDRO_ISL100_CONFIG_H_

#include <string>

namespace lidro::isl100 {

enum stream_mode {
  stream_mode_1 = 1,
  stream_mode_2,
  stream_mode_3,
  stream_mode_4,
};

// configuration parameters
struct Config {
  stream_mode mode;
  std::string frame_id;
  std::string host_addr;
  int host_ctrl_port;
  int host_data_port;

  std::string lidar_addr;
  int lidar_ctrl_port;
  double frequency;
  bool publish_ambient_data;
  double min_distance;
  double max_distance;
  int max_intensity;
};

}

#endif // LIDRO_ISL100_CONFIG_H_
