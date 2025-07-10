#ifndef OBSTACLE_MARKER_TEST__DEBUG_OBJECT_HPP_
#define OBSTACLE_MARKER_TEST__DEBUG_OBJECT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <string>
#include <vector>

namespace obstacle_marker_test
{

// 简化的InputChannel结构
struct InputChannel
{
  uint index;               // index of the channel
  std::string input_topic;  // topic name of the detection
  std::string long_name = "Detected Object";  // full name of the detection
  std::string short_name = "DET";             // abbreviation of the name
  bool is_spawn_enabled = true;               // enable spawn of the object
  bool trust_existence_probability = true;    // trust object existence probability
  bool trust_extension = true;                // trust object extension
  bool trust_classification = true;           // trust object classification
  bool trust_orientation = true;              // trust object orientation(yaw)
};

// 简化的ObjectData结构
struct ObjectData
{
  rclcpp::Time time;

  // object uuid
  boost::uuids::uuid uuid;
  std::string uuid_str;

  // association link, pair of coordinates
  // tracker to detection
  geometry_msgs::msg::Point tracker_point;
  geometry_msgs::msg::Point detection_point;
  bool is_associated{false};

  // existence probabilities
  std::vector<float> existence_vector;
  float total_existence_probability;

  // detection channel id
  uint channel_id;
};

class TrackerObjectDebugger
{
public:
  TrackerObjectDebugger(
    const std::string & frame_id, const std::vector<InputChannel> & channels_config);

  void draw(
    const std::vector<std::vector<ObjectData>> & object_data_groups,
    visualization_msgs::msg::MarkerArray & marker_array) const;

private:
  std::string frame_id_;
  const std::vector<InputChannel> channels_config_;

  // Helper function to convert UUID to int
  int uuidToInt(const boost::uuids::uuid & uuid) const;
};

}  // namespace obstacle_marker_test

#endif  // OBSTACLE_MARKER_TEST__DEBUG_OBJECT_HPP_
