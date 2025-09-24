/**
 * @file bag_timestamp_analyzer.cpp
 * @brief Extract and output timestamps for every message in a specified topic from a ROS2 bag file
 */

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>
#include <filesystem>

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

class BagTimestampAnalyzer
{
public:
    struct Config {
        std::string input_bag_path;
        std::string target_topic;
        bool verbose = false;
        bool human_readable = true;
        bool show_message_count = true;
    };
    
    explicit BagTimestampAnalyzer(Config config)
        : config_(std::move(config))
    {
        if (config_.verbose) {
            std::cout << "Initialized Bag Timestamp Analyzer:\n"
                      << "  Input bag: " << config_.input_bag_path << "\n"
                      << "  Target topic: " << config_.target_topic << "\n"
                      << "  Human readable: " << (config_.human_readable ? "yes" : "no") << "\n";
        }
    }
    
    [[nodiscard]] auto analyze() const -> bool {
        // Check if input bag file exists
        if (!fs::exists(config_.input_bag_path)) {
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
        
        // Process messages and extract timestamps
        size_t message_count = 0;
        size_t target_topic_count = 0;
        
        std::cout << "Analyzing timestamps for topic: " << config_.target_topic << std::endl;
        std::cout << "Format: " << (config_.human_readable ? "human-readable" : "nanoseconds") << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        
        while (reader.has_next()) {
            auto bag_message = reader.read_next();
            message_count++;
            
            if (bag_message->topic_name == config_.target_topic) {
                target_topic_count++;
                
                if (config_.human_readable) {
                    std::string formatted_time = formatTimestamp(bag_message->time_stamp);
                    std::cout << "Message " << target_topic_count << ": " << formatted_time << std::endl;
                } else {
                    std::cout << "Message " << target_topic_count << ": " << bag_message->time_stamp << " ns" << std::endl;
                }
            }
            
            if (config_.verbose && message_count % 1000 == 0) {
                std::cout << "Processed " << message_count << " total messages..." << std::endl;
            }
        }
        
        std::cout << "----------------------------------------" << std::endl;
        if (config_.show_message_count) {
            std::cout << "Total messages in bag: " << message_count << std::endl;
            std::cout << "Messages in topic '" << config_.target_topic << "': " << target_topic_count << std::endl;
        }
        
        return true;
    }

private:
    Config config_;
};

auto print_usage(const char* program_name) -> void {
    std::cout << "Usage: " << program_name << " <bag_file> <topic_name> [options]" << std::endl;
    std::cout << "  bag_file:   Path to the ROS2 bag file (.mcap)" << std::endl;
    std::cout << "  topic_name: Name of the topic to analyze" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --raw       Output timestamps in nanoseconds (raw format)" << std::endl;
    std::cout << "  --verbose   Enable verbose output" << std::endl;
    std::cout << "  --help      Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program_name << " data.bag /sensor/lidar" << std::endl;
    std::cout << "  " << program_name << " data.bag /camera/image --raw" << std::endl;
    std::cout << "  " << program_name << " data.bag /imu/data --verbose" << std::endl;
}

auto main(int argc, char** argv) -> int {
    // Parse command line arguments
    if (argc < 3) {
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
    std::string topic_name = argv[2];
    
    BagTimestampAnalyzer::Config config;
    config.input_bag_path = bag_file;
    config.target_topic = topic_name;
    config.human_readable = true;
    config.verbose = false;
    config.show_message_count = true;
    
    // Parse optional flags
    for (int i = 3; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--raw") {
            config.human_readable = false;
        } else if (arg == "--verbose") {
            config.verbose = true;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }
    
    try {
        BagTimestampAnalyzer analyzer(config);
        bool success = analyzer.analyze();
        return success ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
