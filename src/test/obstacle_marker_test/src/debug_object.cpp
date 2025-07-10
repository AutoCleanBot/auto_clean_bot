#include "obstacle_marker_test/debug_object.hpp"
#include <std_msgs/msg/color_rgba.hpp>

namespace obstacle_marker_test {

TrackerObjectDebugger::TrackerObjectDebugger(const std::string &frame_id,
                                             const std::vector<InputChannel> &channels_config)
    : frame_id_(frame_id), channels_config_(channels_config) {}

int TrackerObjectDebugger::uuidToInt(const boost::uuids::uuid &uuid) const {
    // Simple hash function to convert UUID to int
    std::size_t hash = boost::hash<boost::uuids::uuid>{}(uuid);
    return static_cast<int>(hash & 0x7FFFFFFF); // Ensure positive int
}

void TrackerObjectDebugger::draw(const std::vector<std::vector<ObjectData>> &object_data_groups,
                                 visualization_msgs::msg::MarkerArray &marker_array) const {
    // initialize markers
    marker_array.markers.clear();

    constexpr int PALETTE_SIZE = 16;
    constexpr std::array<std::array<double, 3>, PALETTE_SIZE> color_array = {{
        {{0.0, 0.0, 1.0}},    // Blue
        {{0.0, 1.0, 0.0}},    // Green
        {{1.0, 1.0, 0.0}},    // Yellow
        {{1.0, 0.0, 0.0}},    // Red
        {{0.0, 1.0, 1.0}},    // Cyan
        {{1.0, 0.0, 1.0}},    // Magenta
        {{1.0, 0.64, 0.0}},   // Orange
        {{0.75, 1.0, 0.0}},   // Lime
        {{0.0, 0.5, 0.5}},    // Teal
        {{0.5, 0.0, 0.5}},    // Purple
        {{1.0, 0.75, 0.8}},   // Pink
        {{0.65, 0.17, 0.17}}, // Brown
        {{0.5, 0.0, 0.0}},    // Maroon
        {{0.5, 0.5, 0.0}},    // Olive
        {{0.0, 0.0, 0.5}},    // Navy
        {{0.5, 0.5, 0.5}}     // Grey
    }};

    for (const auto &object_data_group : object_data_groups) {
        if (object_data_group.empty())
            continue;
        const auto object_data_front = object_data_group.front();
        const auto object_data_back = object_data_group.back();

        // set a reference marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = object_data_front.time;
        marker.id = uuidToInt(object_data_front.uuid);
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0; // white
        marker.lifetime = rclcpp::Duration::from_seconds(0);

        // get marker - existence_probability
        visualization_msgs::msg::Marker text_marker;
        text_marker = marker;
        text_marker.ns = "existence_probability";
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.z += 1.8;
        text_marker.scale.z = 0.5;
        text_marker.pose.position.x = object_data_front.tracker_point.x;
        text_marker.pose.position.y = object_data_front.tracker_point.y;
        text_marker.pose.position.z = object_data_front.tracker_point.z + 2.5;

        // show the last existence probability
        // print existence probability with channel name
        // probability to text, two digits of percentage
        std::string existence_probability_text = "";

        // total probability
        existence_probability_text += "total:";
        existence_probability_text +=
            std::to_string(static_cast<int>(object_data_front.total_existence_probability * 100)) + "\n";

        // probability per channel
        const size_t channel_size = channels_config_.size();
        for (size_t i = 0; i < channel_size; ++i) {
            if (object_data_front.existence_vector[i] < 0.00101)
                continue;
            existence_probability_text +=
                channels_config_[i].short_name +
                std::to_string(static_cast<int>(object_data_front.existence_vector[i] * 100)) + ":";
        }
        if (!existence_probability_text.empty()) {
            existence_probability_text.pop_back();
        }
        existence_probability_text += "\n" + object_data_front.uuid_str.substr(0, 6);

        text_marker.text = existence_probability_text;

        // loop for each object_data in the group
        // boxed to tracker positions
        // and link lines to the detected positions
        const double marker_height_offset = 1.0;
        const double assign_height_offset = 0.6;

        visualization_msgs::msg::Marker marker_track_boxes;
        marker_track_boxes = marker;
        marker_track_boxes.ns = "track_boxes";
        marker_track_boxes.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker_track_boxes.action = visualization_msgs::msg::Marker::ADD;
        marker_track_boxes.scale.x = 0.4;
        marker_track_boxes.scale.y = 0.4;
        marker_track_boxes.scale.z = 0.4;
        marker_track_boxes.color.a = 0.9;
        marker_track_boxes.color.r = 1.0;
        marker_track_boxes.color.g = 1.0;
        marker_track_boxes.color.b = 1.0;

        // make detected object markers per channel
        std::vector<visualization_msgs::msg::Marker> marker_detect_boxes_per_channel;
        std::vector<visualization_msgs::msg::Marker> marker_detect_lines_per_channel;

        for (size_t idx = 0; idx < channels_config_.size(); idx++) {
            // get color - by channel index
            std_msgs::msg::ColorRGBA color;
            color.a = 0.9;
            color.r = color_array[idx % PALETTE_SIZE][0];
            color.g = color_array[idx % PALETTE_SIZE][1];
            color.b = color_array[idx % PALETTE_SIZE][2];

            visualization_msgs::msg::Marker marker_detect_boxes;
            marker_detect_boxes = marker;
            marker_detect_boxes.ns = "detect_boxes_" + channels_config_[idx].short_name;
            marker_detect_boxes.type = visualization_msgs::msg::Marker::CUBE_LIST;
            marker_detect_boxes.action = visualization_msgs::msg::Marker::ADD;
            marker_detect_boxes.scale.x = 0.2;
            marker_detect_boxes.scale.y = 0.2;
            marker_detect_boxes.scale.z = 0.2;
            marker_detect_boxes.color = color;
            marker_detect_boxes_per_channel.push_back(marker_detect_boxes);

            visualization_msgs::msg::Marker marker_lines;
            marker_lines = marker;
            marker_lines.ns = "association_lines_" + channels_config_[idx].short_name;
            marker_lines.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker_lines.action = visualization_msgs::msg::Marker::ADD;
            marker_lines.scale.x = 0.15;
            marker_lines.points.clear();
            marker_lines.color = color;
            marker_detect_lines_per_channel.push_back(marker_lines);
        }

        bool is_associated = false;
        for (const auto &object_data : object_data_group) {
            int channel_id = object_data.channel_id;

            // set box
            geometry_msgs::msg::Point box_point;
            box_point.x = object_data.tracker_point.x;
            box_point.y = object_data.tracker_point.y;
            box_point.z = object_data.tracker_point.z + marker_height_offset;
            marker_track_boxes.points.push_back(box_point);

            // set association marker, if exists
            if (!object_data.is_associated)
                continue;
            is_associated = true;

            // associated object box
            visualization_msgs::msg::Marker &marker_detect_boxes = marker_detect_boxes_per_channel.at(channel_id);
            box_point.x = object_data.detection_point.x;
            box_point.y = object_data.detection_point.y;
            box_point.z = object_data.detection_point.z + marker_height_offset + assign_height_offset;
            marker_detect_boxes.points.push_back(box_point);

            // association line
            visualization_msgs::msg::Marker &marker_lines = marker_detect_lines_per_channel.at(channel_id);
            geometry_msgs::msg::Point line_point;
            line_point.x = object_data.tracker_point.x;
            line_point.y = object_data.tracker_point.y;
            line_point.z = object_data.tracker_point.z + marker_height_offset;
            marker_lines.points.push_back(line_point);
            line_point.x = object_data.detection_point.x;
            line_point.y = object_data.detection_point.y;
            line_point.z = object_data.detection_point.z + marker_height_offset + assign_height_offset;
            marker_lines.points.push_back(line_point);
        }

        // add markers
        for (size_t i = 0; i < channels_config_.size(); i++) {
            if (marker_detect_boxes_per_channel.at(i).points.empty()) {
                marker_detect_boxes_per_channel.at(i).action = visualization_msgs::msg::Marker::DELETE;
            }
            marker_array.markers.push_back(marker_detect_boxes_per_channel.at(i));
        }
        for (size_t i = 0; i < channels_config_.size(); i++) {
            if (marker_detect_lines_per_channel.at(i).points.empty()) {
                marker_detect_lines_per_channel.at(i).action = visualization_msgs::msg::Marker::DELETE;
            }
            marker_array.markers.push_back(marker_detect_lines_per_channel.at(i));
        }

        // if not associated, gray out the track box and text
        if (!is_associated) {
            marker_track_boxes.color.r = 0.5;
            marker_track_boxes.color.g = 0.5;
            marker_track_boxes.color.b = 0.5;
            marker_track_boxes.color.a = 0.8;
            text_marker.color.r = 0.5;
            text_marker.color.g = 0.5;
            text_marker.color.b = 0.5;
            text_marker.color.a = 0.9;
        }
        marker_array.markers.push_back(text_marker);
        marker_array.markers.push_back(marker_track_boxes);
    }

    return;
}

} // namespace obstacle_marker_test
