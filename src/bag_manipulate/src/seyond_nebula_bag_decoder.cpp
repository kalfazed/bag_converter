/**
 * @file seyond_nebula_bag_decoder.cpp
 * @brief Decode Seyond LiDAR nebula packets from rosbag and convert to point clouds
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <nebula_msgs/msg/nebula_packet.hpp>

#include <seyond_nebula_decoder/seyond_nebula_decoder.hpp>

#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>
#include <fstream>
#include <map>

namespace fs = std::filesystem;

// Helper function to convert Unix timestamp to human-readable format
std::string formatTimestamp(uint64_t timestamp_ns) {
    // Convert nanoseconds to seconds
    uint64_t timestamp_sec = timestamp_ns / 1000000000;
    uint64_t nanoseconds = timestamp_ns % 1000000000;
    
    // Convert to time_t
    std::time_t time_t_value = static_cast<std::time_t>(timestamp_sec);
    
    // Convert to tm structure
    std::tm* tm_ptr = std::gmtime(&time_t_value);
    if (!tm_ptr) {
        return "Invalid timestamp";
    }
    
    // Format nanoseconds as xxx:yyy:zzz (milliseconds:microseconds:nanoseconds)
    uint64_t milliseconds = nanoseconds / 1000000;
    uint64_t microseconds = (nanoseconds % 1000000) / 1000;
    uint64_t remaining_nanoseconds = nanoseconds % 1000;
    
    // Format as year-month-day:hour-minutes-sec:xxx:yyy:zzz
    std::ostringstream oss;
    oss << std::put_time(tm_ptr, "%Y-%m-%d:%H-%M-%S");
    oss << ":" << std::setfill('0') << std::setw(3) << milliseconds;
    oss << ":" << std::setfill('0') << std::setw(3) << microseconds;
    oss << ":" << std::setfill('0') << std::setw(3) << remaining_nanoseconds;
    
    return oss.str();
}

class SeyondNebulaBagDecoder
{
public:
  struct Config {
    std::string input_bag_path;
    std::string output_bag_path;
    std::string nebula_packets_topic = "";  // Empty = auto-detect all nebula topics
    std::string pointcloud_topic = "";      // Empty = auto-generate from input topic
    
    // Decoder configuration
    std::string sensor_model = "Falcon_Kinetic";
    std::string return_mode = "Dual";
    std::string frame_id = "lidar_top";
    double min_range = 0.3;
    double max_range = 200.0;
    int coordinate_mode = 3;
    bool use_reflectance = true;
    std::string calibration_file = "";
    
    // Merging configuration
    double timestamp_tolerance_ms = 50.0;  // Tolerance in milliseconds for merging pointclouds
    std::string merged_topic_name = "/sensing/lidar/concatenated/pointcloud2";
    std::string merged_frame_id = "base_link";

    bool verbose = false;
  };
  
  explicit SeyondNebulaBagDecoder(const Config& config)
    : config_(config)
  {
    if (config_.verbose) {
      std::cout << "Initialized Seyond Nebula Bag Decoder:\n"
                << "  Input: " << config_.input_bag_path << "\n"
                << "  Output: " << config_.output_bag_path << "\n";
      if (!config_.nebula_packets_topic.empty()) {
        std::cout << "  Nebula packets topic: " << config_.nebula_packets_topic << "\n"
                  << "  Output pointcloud topic: " << config_.pointcloud_topic << "\n";
      } else {
        std::cout << "  Auto-detecting Nebula topics...\n";
      }
      std::cout << "  Sensor model: " << config_.sensor_model << "\n"
                << "  Min range: " << config_.min_range << " m\n"
                << "  Max range: " << config_.max_range << " m\n";
    }
  }
  
  ~SeyondNebulaBagDecoder() {
    // Close all debug files
    for (auto& [frame_id, file_stream] : debug_files_) {
      if (file_stream.is_open()) {
        file_stream.close();
        std::cout << "Closed debug file: " << frame_id << "_debug.txt" << std::endl;
      }
    }
  }
  
  bool process()
  {
    // Check input file exists
    if (!fs::exists(config_.input_bag_path)) {
      std::cerr << "Error: Input bag file does not exist: " << config_.input_bag_path << std::endl;
      return false;
    }
    
    // Open input bag
    rosbag2_storage::StorageOptions storage_options_in;
    storage_options_in.uri = config_.input_bag_path;
    storage_options_in.storage_id = "mcap";
    
    rosbag2_cpp::Reader reader;
    try {
      reader.open(storage_options_in);
    } catch (const std::exception& e) {
      std::cerr << "Error opening input bag: " << e.what() << std::endl;
      return false;
    }
    
    // Get bag metadata
    const auto metadata = reader.get_metadata();
    
    // Discover all nebula_packets topics to convert
    std::map<std::string, std::string> nebula_topic_mapping;  // input_topic -> output_topic
    std::map<std::string, std::unique_ptr<seyond_nebula_decoder::SeyondNebulaDecoder>> decoders;
    
    std::cout << "Scanning for Nebula packet topics..." << std::endl;
    
    for (const auto& topic_info : metadata.topics_with_message_count) {
      const auto& topic_metadata = topic_info.topic_metadata;
      
      // Check if this is a nebula packets topic
      if (topic_metadata.type == "nebula_msgs/msg/NebulaPackets" &&
          topic_metadata.name.find("/nebula_packets") != std::string::npos) {
        
        // Generate converted topic name (replace nebula_packets with pointcloud)
        std::string converted_topic = topic_metadata.name;
        size_t pos = converted_topic.find("/nebula_packets");
        if (pos != std::string::npos) {
          converted_topic.replace(pos, 15, "/nebula_points");  // 15 is length of "/nebula_packets"
        }
        
        nebula_topic_mapping[topic_metadata.name] = converted_topic;
        
        // Create decoder for this topic with appropriate frame_id
        seyond_nebula_decoder::DecoderConfig decoder_config;
        decoder_config.sensor_model = config_.sensor_model;
        decoder_config.return_mode = config_.return_mode;
        decoder_config.min_range = config_.min_range;
        decoder_config.max_range = config_.max_range;
        decoder_config.calibration_file = config_.calibration_file;
        
        // Extract sensor name from topic (e.g., /sensing/lidar/top/nebula_packets -> lidar_top)
        size_t last_slash = topic_metadata.name.rfind("/nebula_packets");
        if (last_slash != std::string::npos && last_slash > 0) {
          size_t second_last_slash = topic_metadata.name.rfind('/', last_slash - 1);
          if (second_last_slash != std::string::npos) {
            std::string sensor_name = topic_metadata.name.substr(second_last_slash + 1, 
                                                         last_slash - second_last_slash - 1);
            decoder_config.frame_id = "lidar_" + sensor_name;
          }
        }
        
        decoders[topic_metadata.name] = std::make_unique<seyond_nebula_decoder::SeyondNebulaDecoder>(decoder_config);
        
        std::cout << "Found Nebula topic: " << topic_metadata.name 
                  << " -> " << converted_topic 
                  << " (frame_id: " << decoder_config.frame_id << ", "
                  << topic_info.message_count << " messages)" << std::endl;
      }
    }
    
    if (nebula_topic_mapping.empty()) {
      // Fall back to single topic mode if explicitly specified
      if (!config_.nebula_packets_topic.empty()) {
        bool topic_found = false;
        for (const auto& topic : metadata.topics_with_message_count) {
          if (topic.topic_metadata.name == config_.nebula_packets_topic) {
            topic_found = true;
            nebula_topic_mapping[config_.nebula_packets_topic] = config_.pointcloud_topic;
            
            seyond_nebula_decoder::DecoderConfig decoder_config;
            decoder_config.sensor_model = config_.sensor_model;
            decoder_config.return_mode = config_.return_mode;
            decoder_config.min_range = config_.min_range;
            decoder_config.max_range = config_.max_range;
            decoder_config.calibration_file = config_.calibration_file;
            decoder_config.frame_id = config_.frame_id;
            decoders[config_.nebula_packets_topic] = 
              std::make_unique<seyond_nebula_decoder::SeyondNebulaDecoder>(decoder_config);
            
            std::cout << "Using specified topic: " << config_.nebula_packets_topic 
                      << " -> " << config_.pointcloud_topic 
                      << " (" << topic.message_count << " messages)" << std::endl;
            break;
          }
        }
        
        if (!topic_found) {
          std::cerr << "Error: Topic " << config_.nebula_packets_topic 
                    << " not found in bag" << std::endl;
          std::cout << "Available topics:" << std::endl;
          for (const auto& topic : metadata.topics_with_message_count) {
            std::cout << "  - " << topic.topic_metadata.name 
                      << " (" << topic.message_count << " messages)" << std::endl;
          }
          return false;
        }
      } else {
        std::cout << "No Nebula packet topics found in the input bag!" << std::endl;
        std::cout << "Looking for topics containing '/nebula_packets' with type 'nebula_msgs/msg/NebulaPackets'" << std::endl;
        return false;
      }
    } else {
      std::cout << "\nFound " << nebula_topic_mapping.size() << " Nebula topic(s) to convert" << std::endl;
    }
    
    // Prepare output bag
    rosbag2_storage::StorageOptions storage_options_out;
    storage_options_out.uri = config_.output_bag_path;
    storage_options_out.storage_id = "mcap";
    
    rosbag2_cpp::Writer writer;
    try {
      writer.open(storage_options_out);
    } catch (const std::exception& e) {
      std::cerr << "Error opening output bag: " << e.what() << std::endl;
      return false;
    }
    
    // Create topics in output bag
    for (const auto& topic_info : metadata.topics_with_message_count) {
      const auto& topic_metadata = topic_info.topic_metadata;
            
      // Check if this is a nebula topic to convert
      auto it = nebula_topic_mapping.find(topic_metadata.name);
      if (it != nebula_topic_mapping.end()) {
        // Create converted point cloud topic
        rosbag2_storage::TopicMetadata pointcloud_topic_meta;
        pointcloud_topic_meta.name = it->second;
        pointcloud_topic_meta.type = "sensor_msgs/msg/PointCloud2";
        pointcloud_topic_meta.serialization_format = "cdr";
        writer.create_topic(pointcloud_topic_meta);
        
        // Also keep the original nebula packets topic with its QoS
        rosbag2_storage::TopicMetadata nebula_topic_meta;
        nebula_topic_meta.name = topic_metadata.name;
        nebula_topic_meta.type = topic_metadata.type;
        nebula_topic_meta.serialization_format = topic_metadata.serialization_format;
        nebula_topic_meta.offered_qos_profiles = topic_metadata.offered_qos_profiles;
        writer.create_topic(nebula_topic_meta);
      } else {
        // Not a nebula topic, copy as-is
        rosbag2_storage::TopicMetadata new_topic_metadata;
        new_topic_metadata.name = topic_metadata.name;
        new_topic_metadata.type = topic_metadata.type;
        new_topic_metadata.serialization_format = topic_metadata.serialization_format;
        new_topic_metadata.offered_qos_profiles = topic_metadata.offered_qos_profiles;
        writer.create_topic(new_topic_metadata);
      }
    }
    
    // Create merged pointcloud topic if we have multiple lidars
    if (nebula_topic_mapping.size() > 1) {
      rosbag2_storage::TopicMetadata merged_topic_meta;
      merged_topic_meta.name = config_.merged_topic_name;
      merged_topic_meta.type = "sensor_msgs/msg/PointCloud2";
      merged_topic_meta.serialization_format = "cdr";
      writer.create_topic(merged_topic_meta);
      std::cout << "Created merged pointcloud topic: " << config_.merged_topic_name << std::endl;
    }
    
    // Process messages
    rclcpp::Serialization<nebula_msgs::msg::NebulaPackets> nebula_serializer;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serializer;
    rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serializer;
    
    size_t message_count = 0;
    size_t packets_processed = 0;
    size_t clouds_generated = 0;
    size_t calibration_packets = 0;
    std::map<std::string, size_t> topic_conversion_counts;
    
    while (reader.has_next()) {
      auto bag_message = reader.read_next();
      message_count++;
      
      // Check if this is a tf_static topic
      if (bag_message->topic_name == "/tf_static") {
        // Process TF static transforms
        try {
          rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
          tf2_msgs::msg::TFMessage tf_msg;
          tf_serializer.deserialize_message(&serialized_msg, &tf_msg);
          
          // Store transforms for later use
          for (const auto& transform : tf_msg.transforms) {
            std::string child_frame = transform.child_frame_id;
            std::string parent_frame = transform.header.frame_id;
            
            // Create tf2::Transform from the message
            tf2::Vector3 translation(
              transform.transform.translation.x,
              transform.transform.translation.y,
              transform.transform.translation.z
            );
            tf2::Quaternion rotation(
              transform.transform.rotation.x,
              transform.transform.rotation.y,
              transform.transform.rotation.z,
              transform.transform.rotation.w
            );
            tf2::Transform tf_transform(rotation, translation);
            
            // Store the transform from child_frame to parent_frame
            tf_transforms_[child_frame] = tf_transform;
            
            if (config_.verbose) {
              std::cout << "Stored TF transform: " << child_frame << " -> " << parent_frame << std::endl;
            }
          }
        } catch (const std::exception& e) {
          std::cerr << "Error processing TF static message: " << e.what() << std::endl;
        }
        
        // Copy original TF message to output
        writer.write(bag_message);
      }
      // Check if this is a nebula topic to convert
      else if (auto it = nebula_topic_mapping.find(bag_message->topic_name); it != nebula_topic_mapping.end()) {
        // Copy original nebula packets message to output
        writer.write(bag_message);
        
        // Process nebula packets
        // Deserialize nebula_msgs
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        nebula_msgs::msg::NebulaPackets nebula_msgs;
        nebula_serializer.deserialize_message(&serialized_msg, &nebula_msgs);
        
        packets_processed++;
        topic_conversion_counts[bag_message->topic_name]++;
        
        // Get decoder for this topic
        auto& decoder = decoders[bag_message->topic_name];
        
        // Decode packets to point cloud directly using nebula_msgs
        auto nebula_cloud = decoder->ConvertNebulaPackets(nebula_msgs);
        
        if (nebula_cloud && !nebula_cloud->empty()) {
          // Convert to PointCloud2 message
          sensor_msgs::msg::PointCloud2 pc2_msg;
          
          // Nebula uses PointXYZIRCAEDT, need to convert
          pcl::PointCloud<pcl::PointXYZI> simple_cloud;
          simple_cloud.header = nebula_cloud->header;
          simple_cloud.width = nebula_cloud->width;
          simple_cloud.height = nebula_cloud->height;
          simple_cloud.is_dense = nebula_cloud->is_dense;
          
          for (const auto& pt : nebula_cloud->points) {
            pcl::PointXYZI simple_pt;
            simple_pt.x = pt.x;
            simple_pt.y = pt.y;
            simple_pt.z = pt.z;
            simple_pt.intensity = pt.intensity;
            simple_cloud.push_back(simple_pt);
          }
          
          pcl::toROSMsg(simple_cloud, pc2_msg);
          
          // Set header
          pc2_msg.header.stamp.sec = bag_message->time_stamp / 1000000000;
          pc2_msg.header.stamp.nanosec = bag_message->time_stamp % 1000000000;
          pc2_msg.header.frame_id = nebula_msgs.header.frame_id.empty() ? 
                                     decoder->GetConfig().frame_id : nebula_msgs.header.frame_id;
         
          // debug the timestamp of the timestamp of this pc2_msg and original bag_message
          std::cout << "================================================" << std::endl;
          std::cout << "frame_id: " << pc2_msg.header.frame_id << std::endl;
          std::cout << "generated simple_cloud size: " << simple_cloud.points.size() << std::endl;
          std::cout << "Number of packets processed: " << packets_processed << std::endl;
          std::cout << "Number of clouds generated: " << clouds_generated << std::endl;
          std::cout << "pc2_msg timestamp (human-readable): " << formatTimestamp(bag_message->time_stamp) << std::endl;
          std::cout << "================================================" << std::endl;
          
          // Write debug information to frame_id specific file
          if (config_.verbose) {
            std::ofstream& debug_file = getDebugFile(pc2_msg.header.frame_id);
            debug_file << pc2_msg.header.frame_id << ": " << formatTimestamp(bag_message->time_stamp) << std::endl;
            debug_file.flush(); // Ensure data is written immediately
          }

          // Store pointcloud for merging if we have multiple lidars
          if (nebula_topic_mapping.size() > 1) {
            timestamped_pointclouds_[bag_message->time_stamp][pc2_msg.header.frame_id] = pc2_msg;
          }
          
          // Serialize and write to bag
          rclcpp::SerializedMessage serialized_pc2;
          pc2_serializer.serialize_message(&pc2_msg, &serialized_pc2);
          
          auto pc2_bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
          pc2_bag_msg->topic_name = it->second;  // Use mapped output topic name
          pc2_bag_msg->time_stamp = bag_message->time_stamp;
          pc2_bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>(
            serialized_pc2.release_rcl_serialized_message());
          
          writer.write(pc2_bag_msg);
          
          clouds_generated++;
          
          if (config_.verbose && clouds_generated % 10 == 0) {
            std::cout << "Generated " << clouds_generated << " point clouds from "
                      << packets_processed << " packet messages" << std::endl;
          }
        }
        
        // Check for calibration packets
        for (const auto& packet : nebula_msgs.packets) {
          if (packet.data.size() > 40) {
            uint8_t type_byte = packet.data[38];
            if (type_byte == 100) {
              calibration_packets++;
              if (calibration_packets == 1) {
                std::cout << "Found calibration packet in nebula data" << std::endl;
                decoder->SetCalibrationData(packet.data);
              }
            }
          }
        }
      } else {
        // Write other messages as-is
        writer.write(bag_message);
      }
      
      if (message_count % 1000 == 0) {
        std::cout << "Processed " << message_count << " messages, converted " 
                  << packets_processed << " nebula packets" << std::endl;
      }
    }
    // Show the size of the timestamped_pointclouds_
    std::cout << "Size of timestamped_pointclouds_: " << timestamped_pointclouds_.size() << std::endl;
    
    // Process merged pointclouds if we have multiple lidars
    size_t merged_clouds_generated = 0;
    if (nebula_topic_mapping.size() > 1) {
      std::cout << "\nProcessing merged pointclouds with tolerance: " 
                << config_.timestamp_tolerance_ms << "ms..." << std::endl;
      
      // Group pointclouds by time windows
      auto time_groups = groupPointCloudsByTimeWindow();
      
      std::cout << "Created " << time_groups.size() << " time groups from " 
                << timestamped_pointclouds_.size() << " timestamped pointclouds" << std::endl;
      
      for (const auto& group : time_groups) {
        // Check if we have pointclouds from all expected lidars
        if (group.size() == nebula_topic_mapping.size() - 1) {
          // All lidars have data for this time group, merge them
          sensor_msgs::msg::PointCloud2 merged_cloud = concatenatePointClouds(group, config_.merged_frame_id);
          
          // Use the earliest timestamp from the group
          uint64_t earliest_timestamp = UINT64_MAX;
          for (const auto& [frame_id, pc] : group) {
            uint64_t pc_timestamp = pc.header.stamp.sec * 1000000000ULL + pc.header.stamp.nanosec;
            if (pc_timestamp < earliest_timestamp) {
              earliest_timestamp = pc_timestamp;
            }
          }
          
          // Set timestamp
          merged_cloud.header.stamp.sec = earliest_timestamp / 1000000000;
          merged_cloud.header.stamp.nanosec = earliest_timestamp % 1000000000;
          
          // Serialize and write merged pointcloud
          rclcpp::SerializedMessage serialized_merged;
          pc2_serializer.serialize_message(&merged_cloud, &serialized_merged);
          
          auto merged_bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
          merged_bag_msg->topic_name = config_.merged_topic_name;
          merged_bag_msg->time_stamp = earliest_timestamp;
          merged_bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>(
            serialized_merged.release_rcl_serialized_message());
          
          writer.write(merged_bag_msg);
          merged_clouds_generated++;
          std::cout << "merging pointclouds for frame_id: " << merged_clouds_generated << ": timestamp: " << formatTimestamp(earliest_timestamp) << std::endl;

        } else {
          // Some lidars missing data for this time group, skip
          if (config_.verbose) {
            std::cout << "Skipping time group - only " << group.size() << " of " 
                      << nebula_topic_mapping.size() << " lidars have data" << std::endl;
          }
        }
      }
      
      std::cout << "Generated " << merged_clouds_generated << " merged point clouds from "
                << time_groups.size() << " time groups" << std::endl;
    }
   
    std::cout << "\n========== Conversion Summary ==========" << std::endl;
    std::cout << "Total messages processed: " << message_count << std::endl;
    std::cout << "Total Nebula packets processed: " << packets_processed << std::endl;
    std::cout << "Total point clouds generated: " << clouds_generated << std::endl;

    if (nebula_topic_mapping.size() > 1) {
      std::cout << "Total merged point clouds generated: " << merged_clouds_generated << std::endl;
    }

    std::cout << "Calibration packets found: " << calibration_packets << std::endl;
    
    if (!topic_conversion_counts.empty()) {
      std::cout << "\nConversion details by topic:" << std::endl;
      for (const auto& [topic, count] : topic_conversion_counts) {
        std::cout << "  " << topic << ": " << count << " messages" << std::endl;
        std::cout << "    -> " << nebula_topic_mapping[topic] << std::endl;
      }
    }
    std::cout << "Output written to: " << config_.output_bag_path << std::endl;
    std::cout << "========================================" << std::endl;
    
    return true;
  }

private:
  Config config_;
  
  // Debug file streams for each frame_id
  std::map<std::string, std::ofstream> debug_files_;
  
  // Pointcloud message storage for merging
  std::map<uint64_t, std::map<std::string, sensor_msgs::msg::PointCloud2>> timestamped_pointclouds_;
  
  // TF transforms storage
  std::map<std::string, tf2::Transform> tf_transforms_;  // frame_id -> transform to base_link
  
  // Helper function to get or create debug file stream
  std::ofstream& getDebugFile(const std::string& frame_id) {
    auto it = debug_files_.find(frame_id);
    if (it == debug_files_.end()) {
      // Create new file stream
      std::string filename = frame_id + "_debug.txt";
      debug_files_[frame_id] = std::ofstream(filename, std::ios::out | std::ios::app);
      std::cout << "Created debug file: " << filename << std::endl;
    }
    return debug_files_[frame_id];
  }
  
  // Helper function to group pointclouds by time windows
  std::vector<std::map<std::string, sensor_msgs::msg::PointCloud2>> groupPointCloudsByTimeWindow() {
    std::vector<std::map<std::string, sensor_msgs::msg::PointCloud2>> groups;
    
    if (timestamped_pointclouds_.empty()) {
      return groups;
    }
    
    // Convert tolerance from milliseconds to nanoseconds
    uint64_t tolerance_ns = static_cast<uint64_t>(config_.timestamp_tolerance_ms * 1000000);
    
    // Sort timestamps for processing
    std::vector<uint64_t> sorted_timestamps;
    for (const auto& [timestamp, _] : timestamped_pointclouds_) {
      sorted_timestamps.push_back(timestamp);
    }
    std::sort(sorted_timestamps.begin(), sorted_timestamps.end());
    
    for (uint64_t timestamp : sorted_timestamps) {
      const auto& pointclouds = timestamped_pointclouds_[timestamp];
      
      // Check if this timestamp can be merged with any existing group
      bool merged = false;
      for (auto& group : groups) {
        // Check if any pointcloud in this group is within tolerance
        bool within_tolerance = false;
        for (const auto& [frame_id, pc] : group) {
          // Check timestamp difference between the group's pointcloud and current timestamp
          uint64_t pc_timestamp = pc.header.stamp.sec * 1000000000ULL + pc.header.stamp.nanosec;
          uint64_t time_diff = (timestamp > pc_timestamp) ? 
                              timestamp - pc_timestamp : 
                              pc_timestamp - timestamp;
          
          if (time_diff <= tolerance_ns) {
            within_tolerance = true;
            break;
          }
        }
        
        if (within_tolerance) {
          // Merge pointclouds from this timestamp into the group
          for (const auto& [frame_id, pc] : pointclouds) {
            group[frame_id] = pc;
          }
          merged = true;
          break;
        }
      }
      
      if (!merged) {
        // Create a new group
        groups.push_back(pointclouds);
      }
    }
    
    return groups;
  }
  
  // Helper function to transform pointcloud to base_link frame
  sensor_msgs::msg::PointCloud2 transformPointCloudToBaseLink(const sensor_msgs::msg::PointCloud2& input_cloud, const std::string& source_frame) {
    sensor_msgs::msg::PointCloud2 transformed_cloud = input_cloud;
    
    // Check if we have a transform for this frame
    auto tf_it = tf_transforms_.find(source_frame);
    if (tf_it == tf_transforms_.end()) {
      std::cerr << "Warning: No TF transform found for frame " << source_frame << ", using original cloud" << std::endl;
      return input_cloud;
    }
    
    const tf2::Transform& transform = tf_it->second;
    if (config_.verbose) {
      const tf2::Vector3& t = transform.getOrigin();
      const tf2::Quaternion& q = transform.getRotation();
      std::cout << "[transformPointCloudToBaseLink] frame_id: " << source_frame << std::endl;
      std::cout << "  Transform translation: x=" << t.x() << ", y=" << t.y() << ", z=" << t.z() << std::endl;
      std::cout << "  Transform rotation (quaternion): x=" << q.x() << ", y=" << q.y() << ", z=" << q.z() << ", w=" << q.w() << std::endl;
    }

    
    // Convert to PCL for transformation
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(input_cloud, pcl_cloud);
    
    // Transform each point
    for (auto& point : pcl_cloud.points) {
      tf2::Vector3 point_vec(point.x, point.y, point.z);
      tf2::Vector3 transformed_point = transform * point_vec;
      
      point.x = transformed_point.x();
      point.y = transformed_point.y();
      point.z = transformed_point.z();
    }

    // Apply additional fixed transform: translation (0.863100, 0.000000, 1.736750), rotation (0.000000, -0.002500, 0.000000, 0.999997)
    {
      tf2::Vector3 extra_translation(0.863100, 0.000000, 1.736750);
      tf2::Quaternion extra_rotation(0.000000, -0.002500, 0.000000, 0.999997);
      tf2::Transform extra_transform(extra_rotation, extra_translation);

      for (auto& point : pcl_cloud.points) {
        tf2::Vector3 point_vec(point.x, point.y, point.z);
        tf2::Vector3 transformed_point = extra_transform * point_vec;

        point.x = transformed_point.x();
        point.y = transformed_point.y();
        point.z = transformed_point.z();
      }
    }
    
    // Convert back to ROS message
    pcl::toROSMsg(pcl_cloud, transformed_cloud);
    
    // Update frame_id to base_link
    transformed_cloud.header.frame_id = "base_link";
    
    if (config_.verbose) {
      std::cout << "Transformed pointcloud from " << source_frame << " to base_link" << std::endl;
    }
    
    return transformed_cloud;
  }
  
  // Helper function to concatenate pointclouds
  sensor_msgs::msg::PointCloud2 concatenatePointClouds(const std::map<std::string, sensor_msgs::msg::PointCloud2>& pointclouds, const std::string& frame_id) {
    if (pointclouds.empty()) {
      std::cout << "[concatenatePointClouds] No pointclouds to merge." << std::endl;
      return sensor_msgs::msg::PointCloud2{};
    }

    std::cout << "[concatenatePointClouds] Number of sensor_msgs::msg::PointCloud2 to merge: " << pointclouds.size() << std::endl;
    //printout the header.frame_id for each pointcloud
    for (const auto& [frame_id, pc] : pointclouds) {
      std::cout << "[concatenatePointClouds] frame_id: " << frame_id << std::endl;
    }

    // Transform all pointclouds to base_link frame before concatenation
    std::map<std::string, sensor_msgs::msg::PointCloud2> transformed_clouds;
    for (const auto& [frame_id, pc] : pointclouds) {
      sensor_msgs::msg::PointCloud2 transformed_pc = transformPointCloudToBaseLink(pc, frame_id);
      transformed_clouds[frame_id] = transformed_pc;
    }

    // Use the first transformed pointcloud as base
    auto it = transformed_clouds.begin();
    sensor_msgs::msg::PointCloud2 merged = it->second;
    ++it;

    // Concatenate remaining transformed pointclouds
    for (; it != transformed_clouds.end(); ++it) {
      // Convert to PCL for concatenation
      pcl::PointCloud<pcl::PointXYZI> pcl_merged, pcl_current;
      pcl::fromROSMsg(merged, pcl_merged);
      pcl::fromROSMsg(it->second, pcl_current);

      // Concatenate
      pcl_merged += pcl_current;

      // Convert back to ROS message
      pcl::toROSMsg(pcl_merged, merged);
    }

    // Update header with merged frame_id
    merged.header.frame_id = frame_id;

    return merged;
  }
};

int main(int argc, char** argv)
{
  // Parse command line arguments
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <input_bag> <output_bag> [options]\n"
              << "\nThis tool automatically detects and converts all Nebula packet topics.\n"
              << "Topics containing '/nebula_packets' will be converted to '/pointcloud'.\n"
              << "\nOptions:\n"
              << "  --nebula-topic <topic>  : Specific nebula packets topic (auto-detects if not specified)\n"
              << "  --output-topic <topic>  : Output pointcloud topic (auto-generates if not specified)\n"
              << "  --sensor-model <model>  : Sensor model (default: Falcon_Kinetic)\n"
              << "  --return-mode <mode>    : Return mode (default: Dual)\n"
              << "  --frame-id <id>         : Frame ID (default: lidar_top)\n"
              << "  --min-range <meters>    : Minimum range (default: 0.3)\n"
              << "  --max-range <meters>    : Maximum range (default: 200.0)\n"
              << "  --coordinate-mode <int> : Coordinate mode 0-3 (default: 3)\n"
              << "  --calibration <file>    : Calibration file path\n"
              << "  --verbose               : Verbose output\n";
    return 1;
  }
  
  SeyondNebulaBagDecoder::Config config;
  config.input_bag_path = argv[1];
  config.output_bag_path = argv[2];
  
  // Parse optional arguments
  for (int i = 3; i < argc; ++i) {
    std::string arg = argv[i];
    
    if (arg == "--nebula-topic" && i + 1 < argc) {
      config.nebula_packets_topic = argv[++i];
    } else if (arg == "--output-topic" && i + 1 < argc) {
      config.pointcloud_topic = argv[++i];
    } else if (arg == "--sensor-model" && i + 1 < argc) {
      config.sensor_model = argv[++i];
    } else if (arg == "--return-mode" && i + 1 < argc) {
      config.return_mode = argv[++i];
    } else if (arg == "--frame-id" && i + 1 < argc) {
      config.frame_id = argv[++i];
    } else if (arg == "--min-range" && i + 1 < argc) {
      config.min_range = std::stod(argv[++i]);
    } else if (arg == "--max-range" && i + 1 < argc) {
      config.max_range = std::stod(argv[++i]);
    } else if (arg == "--coordinate-mode" && i + 1 < argc) {
      config.coordinate_mode = std::stoi(argv[++i]);
    } else if (arg == "--calibration" && i + 1 < argc) {
      config.calibration_file = argv[++i];
    } else if (arg == "--verbose") {
      config.verbose = true;
    }
  }
  
  // Initialize ROS2 (required for serialization)
  rclcpp::init(argc, argv);
  
  // Process bag
  SeyondNebulaBagDecoder decoder(config);
  bool success = decoder.process();
  
  // Cleanup
  rclcpp::shutdown();
  
  return success ? 0 : 1;
}