#include "lidro/isl100/driver.hpp"

#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include "log_message.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cmath>

namespace lidro::isl100 {

Driver::Driver(const rclcpp::NodeOptions &options): rclcpp::Node("lidro_isl100_node", options) {
  this->declare_parameter<int>("mode", 0);
  this->declare_parameter<std::string>("frame_id", "lidar_link");
  this->declare_parameter<double>("frequency", 30);
  this->declare_parameter<std::string>("host_addr", "192.168.1.100");
  this->declare_parameter<int>("host_ctrl_port", 5051);
  this->declare_parameter<int>("host_data_port", 5055);
  this->declare_parameter<std::string>("lidar_addr", "192.168.1.10");
  this->declare_parameter<int>("lidar_ctrl_port", 1011);
  this->declare_parameter<bool>("publish_ambient_data", false);
  this->declare_parameter<double>("min_distance", 0.0);

  config_.mode = (stream_mode)this->get_parameter("mode").as_int();
  config_.frame_id = this->get_parameter("frame_id").as_string();
  config_.frequency = this->get_parameter("frequency").as_double();
  config_.host_addr = this->get_parameter("host_addr").as_string();
  config_.host_ctrl_port = this->get_parameter("host_ctrl_port").as_int();
  config_.host_data_port = this->get_parameter("host_data_port").as_int();
  config_.lidar_addr = this->get_parameter("lidar_addr").as_string();
  config_.lidar_ctrl_port = this->get_parameter("lidar_ctrl_port").as_int();
  config_.publish_ambient_data = this->get_parameter("publish_ambient_data").as_bool();
  config_.min_distance = this->get_parameter("min_distance").as_double();
  set_param_res_ = this->add_on_set_parameters_callback(std::bind(&Driver::onParamUpdated, this, std::placeholders::_1));

  initialize();
}

Driver::~Driver() {
}

rcl_interfaces::msg::SetParametersResult Driver::onParamUpdated(const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  bool reboot_required{false};
  for (const auto &param : parameters) {
    RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", param.get_type_name().c_str());
    RCLCPP_INFO(this->get_logger(), "%s", param.value_to_string().c_str());

    if (param.get_name() == "stream_mode") {
      config_.mode = (stream_mode)param.as_int();
      if(config_.mode <= 0 || config_.mode > 3) {
        RCLCPP_ERROR(this->get_logger(), "stream-mode: out-of-range-error %d  available:[1,4]\n", (int) config_.mode);
        config_.mode = stream_mode_1;
      } 
      reboot_required = true;
    }

    if (param.get_name() == "frame_id") {
      config_.frame_id = param.as_string();
    }

    if (param.get_name() == "host_addr") {
      config_.host_addr = param.as_string();
      reboot_required = true;
    }

    if (param.get_name() == "host_ctrl_port") {
      config_.host_ctrl_port = param.as_int();
      reboot_required = true;
    }

    if (param.get_name() == "host_data_port") {
      config_.host_data_port = param.as_int();
      reboot_required = true;
    }

    if (param.get_name() == "lidar_addr") {
      config_.lidar_addr = param.as_string();
      reboot_required = true;
    }

    if (param.get_name() == "lidar_ctrl_port") {
      config_.lidar_ctrl_port = param.as_int();
      reboot_required = true;
    }

    if (param.get_name() == "publish_ambient_data") {
      config_.publish_ambient_data = param.as_bool();
    }

    if (param.get_name() == "min_distance") {
      config_.min_distance = param.as_double();
    }
  }

  if (reboot_required) {
    interface_ = nullptr;
    interface_ = std::make_unique<lidro::isl100::Interface>(config_);
    interface_->setOnFrame([this](std::unique_ptr<lidro::isl100::Frame> frame)
                            { return updateFrame(std::move(frame)); });
  }
  return result;
}

void Driver::initialize() {
  ambi_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lidro/image", 10);
  intensity_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lidro/intensity", 10);
  pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidro/points", 10);

  interface_ = std::make_unique<lidro::isl100::Interface>(config_);
  interface_->setOnFrame([this](std::unique_ptr<lidro::isl100::Frame> frame)
                          { return updateFrame(std::move(frame)); });

  clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  last_time_ = clock_->now();
}

int Driver::updateFrame(std::unique_ptr<Frame> frame) {
  int err;
  rclcpp::Time now = clock_->now();
  rclcpp::Duration interval = rclcpp::Duration::from_seconds(1.0 / config_.frequency);

  if (now - last_time_ < interval) return 0;
  last_time_ = now;

  if (config_.publish_ambient_data) {
    err = publishImage(*frame);
    if(err) {
      RCLCPP_ERROR(this->get_logger(), "failed to publish ambient");
    }
  }
    
  err = publish(*frame);
  if(err) {
    RCLCPP_ERROR(this->get_logger(), "frame updated received size: %d", err);
    return err;
  }

  return 0;
}

int Driver::publishImage(Frame &frame) {
  auto mat = frame.ambient_data();

  cv::Mat image(mat.rows(), mat.cols(), CV_8UC1);
  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      image.at<uint8_t>(i, j) = mat(i, j);
    }
  }

  cv::Mat brighterImage;
  int brightness = 60; // You can adjust this value to make the image as bright as needed
  cv::add(image, cv::Scalar(brightness), brighterImage);

  cv::Mat smoothed;
  cv::GaussianBlur(brighterImage, smoothed, cv::Size(5, 5), 0);

  auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", smoothed).toImageMsg();

  message->header.stamp = this->now();
  message->header.frame_id = "lidar_opt_link";

  ambi_pub_->publish(*message);
  return 0;
}

int Driver::publish(Frame &frame) {
  auto distances = frame.distances();
  auto intensities = frame.intensities();

  pcl::PointCloud<pcl::PointXYZI> cloud{};
  cloud.width = frame.width();
  cloud.height = frame.height();
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  const float N = (float)frame.height();

  auto deg_to_rad = [](float deg) -> float {
    return (deg * (M_PI / 180.f));
  };

  auto cm_to_m = [](float cm) -> float {
    return (cm * 1e-2);
  };

  auto resolve_phi = [&](size_t i) -> float {
    return 25.f*((i - N/2)/N) + 25.f/(2*N);
  };

  auto resolve_theta = [&](size_t i) -> float {
    // d8 : delta theta
    return (-60.f + frame.d8() * i + frame.d8() / 2);
  };

  for (size_t i = 0; i < cloud.height; ++i) {
    float phi = deg_to_rad(
      -resolve_phi(i)
    );

    for (size_t j = 0; j < cloud.width; ++j) {
      float theta = deg_to_rad(
        resolve_theta(j)
      );

      float d = cm_to_m(
        (float)distances(i, j)
      );
      float intensity = (intensities(i, j) / frame.max_intensity()) * 255.f;
      if (std::isgreater(d,frame.max_distance()) || std::isless(d, config_.min_distance)) {
        cloud.points[i * cloud.width + j].x =
        cloud.points[i * cloud.width + j].y =
        cloud.points[i * cloud.width + j].z =
        cloud.points[i * cloud.width + j].intensity = std::numeric_limits<float>::infinity();
      }
      else {
        const auto cos_phi = std::cos(phi);
        cloud.points[i * cloud.width + j].x = static_cast<float>(d * cos_phi * std::sin(theta)); // 𝑑 ∙ cos 𝜙 ∙ sin(𝜃)
        cloud.points[i * cloud.width + j].y = static_cast<float>(d * cos_phi * std::cos(theta)); // 𝑑 ∙ cos 𝜙 ∙ cos(𝜃)
        cloud.points[i * cloud.width + j].z = static_cast<float>(d * std::sin(phi));             // 𝑑 ∙ sin 𝜙)
        cloud.points[i * cloud.width + j].intensity = static_cast<float>(intensity);
      }
    }
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = config_.frame_id;
  cloud_msg.header.stamp = this->now();

  pcl_pub_->publish(cloud_msg);
  return 0;
}

} // namespace lidro_driver

RCLCPP_COMPONENTS_REGISTER_NODE(lidro::isl100::Driver)
