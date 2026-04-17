#include <yaml-cpp/yaml.h>

#include <gz/transport/Node.hh>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/convert.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

struct TopicConfig {
  std::string ros_topic_name;
  std::string gz_topic_name;
  std::string ros_type_name;
  std::string gz_type_name;
  std::string direction;
};

class CameraBridge : public rclcpp::Node {
  public:
  CameraBridge() : Node("ros_gz_camera_bridge") {
    std::string file_path = this->declare_parameter<std::string>("config_file");

    gz_node_ = std::make_shared<gz::transport::Node>();
    LoadConfig(file_path);
    SetupBridges();
  }

  private:
  void LoadConfig(const std::string &file_path) {
    try {
      const YAML::Node config = YAML::LoadFile(file_path);
      for (const auto &topic_config : config) {
        TopicConfig topic;
        topic.ros_topic_name = topic_config["ros_topic_name"].as<std::string>();
        topic.gz_topic_name = topic_config["gz_topic_name"].as<std::string>();
        topic.ros_type_name = topic_config["ros_type_name"].as<std::string>();
        topic.gz_type_name = topic_config["gz_type_name"].as<std::string>();
        topic.direction = topic_config["direction"].as<std::string>();

        topic_configs_.push_back(topic);
      }
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading YAML file: %s",
                   std::string(e.what()).c_str());
    }
  }

  void SetupBridges() {
    for (const auto &topic_config : topic_configs_) {
      if (topic_config.direction != "GZ_TO_ROS") {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid direction, only supports "
                     "GZ_TO_ROS, but got %s, skipping...",
                     topic_config.direction.c_str());
      }
      // Setup GZ to ROS bridge
      RCLCPP_INFO(this->get_logger(), "Setting up GZ to ROS bridge from: %s to: %s",
                  topic_config.gz_topic_name.c_str(), topic_config.ros_topic_name.c_str());
      if (topic_config.ros_type_name == "sensor_msgs/msg/Image") {
        const auto ros_pub = this->create_publisher<sensor_msgs::msg::Image>(
            topic_config.ros_topic_name, rclcpp::SensorDataQoS());
        image_pubs_.push_back(ros_pub);
        gz_node_->Subscribe<gz::msgs::Image>(
            topic_config.gz_topic_name,
            [this, ros_pub](const gz::msgs::Image &gz_msg) { ImageCallback(gz_msg, ros_pub); });
      } else if (topic_config.ros_type_name == "sensor_msgs/msg/CameraInfo") {
        rclcpp::QoS latched_qos(1);
        latched_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        latched_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        const auto ros_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            topic_config.ros_topic_name, latched_qos);
        cam_info_pubs_.push_back(ros_pub);
        gz_node_->Subscribe<gz::msgs::CameraInfo>(
            topic_config.gz_topic_name, [this, ros_pub](const gz::msgs::CameraInfo &gz_msg) {
              CameraInfoCallback(gz_msg, ros_pub);
            });
      } else if (topic_config.ros_type_name == "sensor_msgs/msg/PointCloud2") {
        const auto ros_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            topic_config.ros_topic_name, rclcpp::SensorDataQoS());
        point_cloud_pubs_.push_back(ros_pub);
        gz_node_->Subscribe<gz::msgs::PointCloudPacked>(
            topic_config.gz_topic_name, [this, ros_pub](const gz::msgs::PointCloudPacked &gz_msg) {
              PointCloudCallback(gz_msg, ros_pub);
            });
      } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported ROS message type: %s, skipping...",
                     topic_config.ros_type_name.c_str());
      }
    }
  }

  std::string ToOpticalLink(const std::string &frame_id) const {
    const std::string from = "_link";
    const std::string to = "_optical_link";

    if (frame_id.size() >= from.size() &&
        frame_id.compare(frame_id.size() - from.size(), from.size(), from) == 0) {
      return frame_id.substr(0, frame_id.size() - from.size()) + to;
    } else {
      RCLCPP_WARN(this->get_logger(), "Frame ID does not end with '_link': %s", frame_id.c_str());
      return frame_id;
    }
  }

  void ImageCallback(const gz::msgs::Image &gz_msg,
                     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_pub) const {
    sensor_msgs::msg::Image ros_msg;
    ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
    ros_msg.header.frame_id = ToOpticalLink(ros_msg.header.frame_id);
    ros_pub->publish(ros_msg);
  }

  void CameraInfoCallback(
      const gz::msgs::CameraInfo &gz_msg,
      rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ros_pub) const {
    sensor_msgs::msg::CameraInfo ros_msg;
    ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
    ros_msg.header.frame_id = ToOpticalLink(ros_msg.header.frame_id);
    ros_pub->publish(ros_msg);
  }

  void PointCloudCallback(
      const gz::msgs::PointCloudPacked &gz_msg,
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ros_pub) const {
    sensor_msgs::msg::PointCloud2 ros_msg;
    ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
    ros_pub->publish(ros_msg);
  }

  std::shared_ptr<gz::transport::Node> gz_node_;

  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pubs_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> cam_info_pubs_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> point_cloud_pubs_;
  std::vector<TopicConfig> topic_configs_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraBridge>());
  rclcpp::shutdown();
  return 0;
}