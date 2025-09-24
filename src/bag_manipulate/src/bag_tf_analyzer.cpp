/**
 * @file bag_tf_analyzer.cpp
 * @brief Extract and analyze TF static transforms from /tf_static topic in a ROS2 bag file
 */

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <map>
#include <set>
#include <algorithm>
#include <numeric>

namespace fs = std::filesystem;

// Helper function to convert Unix timestamp to human-readable format
auto formatTimestamp(uint64_t timestamp_ns) -> std::string {
    // Convert nanoseconds to seconds
    uint64_t timestamp_sec = timestamp_ns / 1000000000;
    uint64_t nanoseconds = timestamp_ns % 1000000000;
    
    // Convert to time_t
    auto time_t_value = static_cast<std::time_t>(timestamp_sec);
    
    // Convert to tm structure
    std::tm* tm_ptr = std::gmtime(&time_t_value);
    if (tm_ptr == nullptr) {
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

// Helper function to format quaternion for display
auto formatQuaternion(const geometry_msgs::msg::Quaternion& quat) -> std::string {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "(" << quat.x << ", " << quat.y << ", " << quat.z << ", " << quat.w << ")";
    return oss.str();
}

// Helper function to format vector3 for display
auto formatVector3(const geometry_msgs::msg::Vector3& vec) -> std::string {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
    return oss.str();
}

// Helper function to calculate quaternion magnitude
auto quaternionMagnitude(const geometry_msgs::msg::Quaternion& quat) -> double {
    return std::sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w);
}

// Helper function to calculate vector3 magnitude
auto vector3Magnitude(const geometry_msgs::msg::Vector3& vec) -> double {
    return std::sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

class BagTfAnalyzer
{
public:
    struct Config {
        std::string input_bag_path;
        std::string target_topic = "/tf_static";
        bool verbose = false;
        bool human_readable = true;
        bool show_message_count = true;
        bool show_transform_details = true;
        bool show_statistics = true;
        bool show_frame_tree = true;
    };
    
    struct TransformInfo {
        std::string parent_frame;
        std::string child_frame;
        geometry_msgs::msg::Vector3 translation;
        geometry_msgs::msg::Quaternion rotation;
        uint64_t timestamp;
        size_t message_index;
    };
    
    explicit BagTfAnalyzer(Config config)
        : config_(std::move(config))
    {
        if (config_.verbose) {
            std::cout << "Initialized Bag TF Analyzer:\n"
                      << "  Input bag: " << config_.input_bag_path << "\n"
                      << "  Target topic: " << config_.target_topic << "\n"
                      << "  Human readable: " << (config_.human_readable ? "yes" : "no") << "\n"
                      << "  Show transform details: " << (config_.show_transform_details ? "yes" : "no") << "\n"
                      << "  Show statistics: " << (config_.show_statistics ? "yes" : "no") << "\n"
                      << "  Show frame tree: " << (config_.show_frame_tree ? "yes" : "no") << "\n";
        }
    }
    
    [[nodiscard]] auto analyze() const -> bool {
        // Check if input bag file exists
        if (!std::filesystem::exists(config_.input_bag_path)) {
            std::cerr << "Error: Input bag file does not exist: " << config_.input_bag_path << std::endl;
            return false;
        }
        
        // Open the bag file
        rosbag2_cpp::Reader reader;
        rosbag2_storage::StorageOptions read_storage_options;
        read_storage_options.uri = config_.input_bag_path;
        read_storage_options.storage_id = "mcap";
        
        rosbag2_cpp::ConverterOptions converter_options{
            rmw_get_serialization_format(),
            rmw_get_serialization_format()
        };
        
        try {
            reader.open(read_storage_options, converter_options);
        } catch (const std::exception& e) {
            std::cerr << "Error opening bag file: " << e.what() << std::endl;
            return false;
        }
        
        // Get all topics and check if target topic exists
        auto topics_and_types = reader.get_all_topics_and_types();
        bool topic_found = false;
        std::string topic_type;
        
        for (const auto& topic_metadata : topics_and_types) {
            if (topic_metadata.name == config_.target_topic) {
                topic_found = true;
                topic_type = topic_metadata.type;
                break;
            }
        }
        
        if (!topic_found) {
            std::cerr << "Error: Topic '" << config_.target_topic << "' not found in bag file." << std::endl;
            std::cerr << "Available topics:" << std::endl;
            for (const auto& topic_metadata : topics_and_types) {
                std::cerr << "  - " << topic_metadata.name << " [" << topic_metadata.type << "]" << std::endl;
            }
            return false;
        }
        
        if (config_.verbose) {
            std::cout << "Found topic: " << config_.target_topic << " [" << topic_type << "]" << std::endl;
        }
        
        // Process messages and extract TF transforms
        size_t message_count = 0;
        size_t target_topic_count = 0;
        std::vector<TransformInfo> transforms;
        std::set<std::string> all_frames;
        std::map<std::string, std::set<std::string>> frame_connections; // parent -> children
        
        std::cout << "Analyzing TF static transforms for topic: " << config_.target_topic << std::endl;
        std::cout << "Format: " << (config_.human_readable ? "human-readable" : "nanoseconds") << std::endl;
        std::cout << "========================================" << std::endl;
        
        while (reader.has_next()) {
            auto bag_message = reader.read_next();
            message_count++;
            
            if (bag_message->topic_name == config_.target_topic) {
                target_topic_count++;
                
                // Deserialize the TF message
                try {
                    tf2_msgs::msg::TFMessage tf_msg;
                    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                    rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization;
                    serialization.deserialize_message(&serialized_msg, &tf_msg);
                    
                    // Process each transform in the message
                    for (size_t i = 0; i < tf_msg.transforms.size(); ++i) {
                        const auto& transform = tf_msg.transforms[i];
                        
                        TransformInfo info;
                        info.parent_frame = transform.header.frame_id;
                        info.child_frame = transform.child_frame_id;
                        info.translation = transform.transform.translation;
                        info.rotation = transform.transform.rotation;
                        info.timestamp = bag_message->time_stamp;
                        info.message_index = target_topic_count;
                        
                        transforms.push_back(info);
                        all_frames.insert(info.parent_frame);
                        all_frames.insert(info.child_frame);
                        frame_connections[info.parent_frame].insert(info.child_frame);
                        
                        if (config_.show_transform_details) {
                            std::cout << "\n--- Transform " << transforms.size() << " (Message " << target_topic_count << ") ---" << std::endl;
                            std::cout << "Parent Frame: " << info.parent_frame << std::endl;
                            std::cout << "Child Frame:  " << info.child_frame << std::endl;
                            
                            if (config_.human_readable) {
                                std::string formatted_time = formatTimestamp(info.timestamp);
                                std::cout << "Timestamp:    " << formatted_time << std::endl;
                            } else {
                                std::cout << "Timestamp:    " << info.timestamp << " ns" << std::endl;
                            }
                            
                            std::cout << "Translation:  " << formatVector3(info.translation) << std::endl;
                            std::cout << "Rotation:     " << formatQuaternion(info.rotation) << std::endl;
                            
                            // Calculate and display magnitudes
                            double trans_mag = vector3Magnitude(info.translation);
                            double rot_mag = quaternionMagnitude(info.rotation);
                            std::cout << "Translation magnitude: " << std::fixed << std::setprecision(6) << trans_mag << std::endl;
                            std::cout << "Rotation magnitude:    " << std::fixed << std::setprecision(6) << rot_mag << std::endl;
                            
                            // Check if quaternion is normalized
                            if (std::abs(rot_mag - 1.0) > 0.001) {
                                std::cout << "WARNING: Quaternion is not normalized! (magnitude = " << rot_mag << ")" << std::endl;
                            }
                        }
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error deserializing TF message " << target_topic_count << ": " << e.what() << std::endl;
                }
            }
            
            if (config_.verbose && message_count % 1000 == 0) {
                std::cout << "Processed " << message_count << " total messages..." << std::endl;
            }
        }
        
        std::cout << "\n========================================" << std::endl;
        
        if (config_.show_message_count) {
            std::cout << "Total messages in bag: " << message_count << std::endl;
            std::cout << "Messages in topic '" << config_.target_topic << "': " << target_topic_count << std::endl;
            std::cout << "Total transforms found: " << transforms.size() << std::endl;
        }
        
        if (config_.show_statistics && !transforms.empty()) {
            printStatistics(transforms);
        }
        
        if (config_.show_frame_tree && !frame_connections.empty()) {
            printFrameTree(frame_connections, all_frames);
        }
        
        return true;
    }

private:
    Config config_;
    
    static void printStatistics(const std::vector<TransformInfo>& transforms) {
        std::cout << "\n--- TF Statistics ---" << std::endl;
        
        // Count unique frame pairs
        std::set<std::pair<std::string, std::string>> unique_pairs;
        for (const auto& transform : transforms) {
            unique_pairs.insert({transform.parent_frame, transform.child_frame});
        }
        std::cout << "Unique frame pairs: " << unique_pairs.size() << std::endl;
        
        // Calculate translation statistics
        std::vector<double> translation_magnitudes;
        for (const auto& transform : transforms) {
            translation_magnitudes.push_back(vector3Magnitude(transform.translation));
        }
        
        if (!translation_magnitudes.empty()) {
            std::sort(translation_magnitudes.begin(), translation_magnitudes.end());
            double min_trans = translation_magnitudes.front();
            double max_trans = translation_magnitudes.back();
            double avg_trans = std::accumulate(translation_magnitudes.begin(), translation_magnitudes.end(), 0.0) / translation_magnitudes.size();
            
            std::cout << "Translation magnitudes:" << std::endl;
            std::cout << "  Min: " << std::fixed << std::setprecision(6) << min_trans << std::endl;
            std::cout << "  Max: " << std::fixed << std::setprecision(6) << max_trans << std::endl;
            std::cout << "  Avg: " << std::fixed << std::setprecision(6) << avg_trans << std::endl;
        }
        
        // Check quaternion normalization
        size_t normalized_count = 0;
        for (const auto& transform : transforms) {
            double rot_mag = quaternionMagnitude(transform.rotation);
            if (std::abs(rot_mag - 1.0) <= 0.001) {
                normalized_count++;
            }
        }
        std::cout << "Normalized quaternions: " << normalized_count << "/" << transforms.size() << std::endl;
    }
    
    void printFrameTree(const std::map<std::string, std::set<std::string>>& frame_connections, 
                       const std::set<std::string>& all_frames) const {
        std::cout << "\n--- Frame Tree Structure ---" << std::endl;
        
        // Find root frames (frames that are not children of any other frame)
        std::set<std::string> child_frames;
        for (const auto& [parent, children] : frame_connections) {
            child_frames.insert(children.begin(), children.end());
        }
        
        std::set<std::string> root_frames;
        for (const auto& frame : all_frames) {
            if (child_frames.find(frame) == child_frames.end()) {
                root_frames.insert(frame);
            }
        }
        
        if (root_frames.empty()) {
            std::cout << "No clear root frames found. All frames are interconnected." << std::endl;
            std::cout << "All frames: ";
            for (const auto& frame : all_frames) {
                std::cout << frame << " ";
            }
            std::cout << std::endl;
        } else {
            std::cout << "Root frames: ";
            for (const auto& root : root_frames) {
                std::cout << root << " ";
            }
            std::cout << std::endl;
            
            // Print tree structure
            for (const auto& root : root_frames) {
                printFrameSubtree(root, frame_connections, 0);
            }
        }
    }
    
    void printFrameSubtree(const std::string& frame, 
                          const std::map<std::string, std::set<std::string>>& frame_connections, 
                          int depth) const {
        std::string indent(depth * 2, ' ');
        std::cout << indent << "└─ " << frame << std::endl;
        
        auto iter = frame_connections.find(frame);
        if (iter != frame_connections.end()) {
            for (const auto& child : iter->second) {
                printFrameSubtree(child, frame_connections, depth + 1);
            }
        }
    }
};

auto print_usage(const char* program_name) -> void {
    std::cout << "Usage: " << program_name << " <bag_file> [options]" << std::endl;
    std::cout << "  bag_file:   Path to the ROS2 bag file (.mcap)" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --topic <name>     Target topic name (default: /tf_static)" << std::endl;
    std::cout << "  --raw              Output timestamps in nanoseconds (raw format)" << std::endl;
    std::cout << "  --verbose          Enable verbose output" << std::endl;
    std::cout << "  --no-details       Don't show individual transform details" << std::endl;
    std::cout << "  --no-stats         Don't show statistics" << std::endl;
    std::cout << "  --no-tree          Don't show frame tree structure" << std::endl;
    std::cout << "  --help             Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program_name << " data.bag" << std::endl;
    std::cout << "  " << program_name << " data.bag --topic /tf_static --verbose" << std::endl;
    std::cout << "  " << program_name << " data.bag --raw --no-details" << std::endl;
}

auto main(int argc, char** argv) -> int {
    // Parse command line arguments
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }
    
    // Check for help flag
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
            print_usage(argv[0]);
            return 0;
        }
    }
    
    std::string bag_file = argv[1];
    
    BagTfAnalyzer::Config config;
    config.input_bag_path = bag_file;
    config.target_topic = "/tf_static";
    config.human_readable = true;
    config.verbose = false;
    config.show_message_count = true;
    config.show_transform_details = true;
    config.show_statistics = true;
    config.show_frame_tree = true;
    
    // Parse optional flags
    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--topic" && i + 1 < argc) {
            config.target_topic = argv[++i];
        } else if (arg == "--raw") {
            config.human_readable = false;
        } else if (arg == "--verbose") {
            config.verbose = true;
        } else if (arg == "--no-details") {
            config.show_transform_details = false;
        } else if (arg == "--no-stats") {
            config.show_statistics = false;
        } else if (arg == "--no-tree") {
            config.show_frame_tree = false;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }
    
    try {
        BagTfAnalyzer analyzer(config);
        bool success = analyzer.analyze();
        return success ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}