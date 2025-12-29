/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

using namespace std;

enum class LiDARType : int {
    Unknown = 0,
    Solid   = 1,   // 固态（如 Livox）
    Mech    = 2    // 机械式多线
};

class DataPreprocess
{
public:
    // 改成带线号的点云
    pcl::PointCloud<Common::Point>::Ptr cloud_input_;
    cv::Mat img_input_;
    LiDARType lidar_type_{LiDARType::Unknown};
    LiDARType lidarType() const { return lidar_type_; }

    DataPreprocess(Params &params)
        : cloud_input_(new pcl::PointCloud<Common::Point>)
    {
        string bag_path   = params.bag_path;
        string image_path = params.image_path;
        string lidar_topic = params.lidar_topic;

        // 读图像
        img_input_ = cv::imread(image_path, cv::IMREAD_UNCHANGED);
        if (img_input_.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), "Loading the image %s failed", image_path.c_str());
            return;
        }

        // 先检查包是否存在
        std::fstream file_;
        file_.open(bag_path, ios::in);
        if (!file_)
        {
            RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), "Loading the rosbag %s failed", bag_path.c_str());
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), "Loading the rosbag %s", bag_path.c_str());
        
        rosbag2_cpp::Reader reader;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        storage_options.storage_id = "mcap"; // 可根据实际情况修改，通常 jazzy 默认 mcap 或 sqlite3

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        try {
            reader.open(storage_options, converter_options);
        } catch (const std::exception &e) {
            // 如果 mcap 打开失败，尝试用 sqlite3
            try {
                storage_options.storage_id = "sqlite3";
                reader.open(storage_options, converter_options);
            } catch (const std::exception &e2) {
                RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), "LOADING BAG FAILED: %s", e2.what());
                return;
            }
        }

        auto topics = reader.get_all_topics_and_types();
        string target_topic_type = "";
        for (const auto & t : topics)
        {
            if (t.name == lidar_topic)
            {
                target_topic_type = t.type;
                break;
            }
        }

        while (reader.has_next())
        {
            auto bag_message = reader.read_next();
            if (bag_message->topic_name != lidar_topic) continue;

            // 1) Livox 自定义消息（含 line 字段）
            if (target_topic_type == "livox_ros_driver2/msg/CustomMsg" || target_topic_type == "livox_ros_driver/CustomMsg")
            {
                rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> serialization;
                livox_ros_driver2::msg::CustomMsg livox_msg;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg, &livox_msg);

                lidar_type_ = LiDARType::Solid;
                cloud_input_->reserve(cloud_input_->size() + livox_msg.point_num);
                for (uint32_t i = 0; i < livox_msg.point_num; ++i)
                {
                    Common::Point p;
                    p.x = livox_msg.points[i].x;
                    p.y = livox_msg.points[i].y;
                    p.z = livox_msg.points[i].z;
                    p.ring = static_cast<std::uint16_t>(livox_msg.points[i].line);
                    cloud_input_->push_back(p);
                }
            }
            // 2) 机械雷达 / 通用 PointCloud2
            else if (target_topic_type == "sensor_msgs/msg/PointCloud2" || target_topic_type == "sensor_msgs/PointCloud2")
            {
                rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
                sensor_msgs::msg::PointCloud2 pcl_msg;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg, &pcl_msg);

                // 优先判断是否有 ring 字段
                bool has_ring = false;
                for (const auto &f : pcl_msg.fields)
                {
                    if (f.name == "ring") { has_ring = true; break; }
                }

                // 使用 iterator 安全读取
                sensor_msgs::PointCloud2ConstIterator<float> it_x(pcl_msg, "x");
                sensor_msgs::PointCloud2ConstIterator<float> it_y(pcl_msg, "y");
                sensor_msgs::PointCloud2ConstIterator<float> it_z(pcl_msg, "z");

                // ring 可能不存在：不存在时用 0xFFFF 表示未知
                std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<std::uint16_t>> it_ring_ptr;
                if (has_ring)
                {
                    it_ring_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<std::uint16_t>(pcl_msg, "ring"));
                    lidar_type_ = LiDARType::Mech;
                }
                else
                {
                    lidar_type_ = LiDARType::Solid;
                }

                const size_t n = static_cast<size_t>(pcl_msg.width) * pcl_msg.height;
                cloud_input_->reserve(cloud_input_->size() + n);

                for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z)
                {
                    Common::Point p;
                    p.x = *it_x;
                    p.y = *it_y;
                    p.z = *it_z;

                    if (has_ring)
                    {
                        p.ring = **it_ring_ptr;
                        ++(*it_ring_ptr);
                    }
                    else
                    {
                        p.ring = 0xFFFF; // 未知线号
                    }

                    cloud_input_->push_back(p);
                }
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), "Loaded %ld points from the rosbag.", cloud_input_->size()); 
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif // DATA_PREPROCESS_HPP