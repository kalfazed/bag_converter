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
      merged_topic_meta.name = merged_topic_name_;
      merged_topic_meta.type = "sensor_msgs/msg/PointCloud2";
      merged_topic_meta.serialization_format = "cdr";
      writer.create_topic(merged_topic_meta);
      std::cout << "Created merged pointcloud topic: " << merged_topic_name_ << std::endl;
    }
    
    // Process messages
    rclcpp::Serialization<nebula_msgs::msg::NebulaPackets> nebula_serializer;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serializer;
    
    size_t message_count = 0;
    size_t packets_processed = 0;
    size_t clouds_generated = 0;
    size_t calibration_packets = 0;
    std::map<std::string, size_t> topic_conversion_counts;
    
    while (reader.has_next()) {
      auto bag_message = reader.read_next();
      message_count++;
      
      // Check if this is a nebula topic to convert
      auto it = nebula_topic_mapping.find(bag_message->topic_name);
      if (it != nebula_topic_mapping.end()) {
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
          std::ofstream& debug_file = getDebugFile(pc2_msg.header.frame_id);
          debug_file << pc2_msg.header.frame_id << ": " << formatTimestamp(bag_message->time_stamp) << std::endl;
          debug_file.flush(); // Ensure data is written immediately

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
   
    std::cout << "\n========== Conversion Summary ==========" << std::endl;
    std::cout << "Total messages processed: " << message_count << std::endl;
    std::cout << "Total Nebula packets processed: " << packets_processed << std::endl;
    std::cout << "Total point clouds generated: " << clouds_generated << std::endl;
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
  std::string merged_topic_name_ = "/merged_lidar_points";
  
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
  
  // Helper function to concatenate pointclouds
  sensor_msgs::msg::PointCloud2 concatenatePointClouds(const std::map<std::string, sensor_msgs::msg::PointCloud2>& pointclouds) {
    if (pointclouds.empty()) {
      return sensor_msgs::msg::PointCloud2{};
    }
    
    // Use the first pointcloud as base
    auto it = pointclouds.begin();
    sensor_msgs::msg::PointCloud2 merged = it->second;
    ++it;
    
    // Concatenate remaining pointclouds
    for (; it != pointclouds.end(); ++it) {
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
    merged.header.frame_id = "merged_lidar";
    
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