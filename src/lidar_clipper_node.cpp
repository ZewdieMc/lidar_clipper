#define PCL_NO_PRECOMPILE
#include <pcl/filters/impl/passthrough.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp> // Include IMU message type
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rmw/types.h>
#include <rmw/serialized_message.h>
#include <rcutils/allocator.h>
#include <rclcpp/serialization.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// Define the custom point type
struct EIGEN_ALIGN16 PointXYZIRTRAR
{
    PCL_ADD_POINT4D;       // XYZ coordinates (float32, datatype 7)
    float intensity;       // Intensity (float32, datatype 7)
    uint32_t t;            // Timestamp (uint32, datatype 6)
    uint16_t reflectivity; // Reflectivity (uint16, datatype 4)
    uint16_t ring;         // Laser ring number (uint16, datatype 4)
    uint16_t ambient;      // Ambient light level (uint16, datatype 4)
    uint32_t range;        // Range (float32, datatype 6)

    // Ensure proper memory alignment for Eigen and SSE
    PCL_MAKE_ALIGNED_OPERATOR_NEW
}; // Ensure 16-byte alignment

// Register the custom point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRTRAR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(uint16_t, ring, ring))(uint16_t, ambient, ambient)(std::uint32_t, range, range));

class LidarClipperNode : public rclcpp::Node
{
public:
    LidarClipperNode()
        : Node("lidar_clipper_node")
    {
        // Define the input and output bag paths
        input_bag_path_ = "/home/zed/thesis_ws/src/bagFiles/saxion_around";    // Update this path
        output_bag_path_ = "/home/zed/thesis_ws/src/bagFiles/saxion_fig8path"; // Update this path

        // Define the vertical clipping range (in meters)
        min_z_ = -30.0;
        max_z_ = 10.0;

        // Open the input bag
        bag_reader_.open(input_bag_path_);

        // Create a writer for the output bag
        bag_writer_.open(output_bag_path_);

        // Create topics in the output bag
        create_topics();

        // Process the bag
        processBag();
    }

private:
    void create_topics()
    {
        // Create the PointCloud2 topic
        rosbag2_storage::TopicMetadata pointcloud_topic;
        pointcloud_topic.name = "/ouster/points";
        pointcloud_topic.type = "sensor_msgs/msg/PointCloud2";
        pointcloud_topic.serialization_format = "cdr";
        bag_writer_.create_topic(pointcloud_topic);

        // Create the IMU topic
        rosbag2_storage::TopicMetadata imu_topic;
        imu_topic.name = "/ouster/imu";
        imu_topic.type = "sensor_msgs/msg/Imu";
        imu_topic.serialization_format = "cdr";
        bag_writer_.create_topic(imu_topic);
    }

    void processBag()
    {
        // Create a serialization object for messages
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pointcloud_serialization;
        rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;

        while (bag_reader_.has_next())
        {
            auto bag_message = bag_reader_.read_next();

            if (!bag_message)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to read next message from bag");
                break;
            }

            if (bag_message->topic_name == "/ouster/points")
            {
                // Deserialize the message into a PointCloud2
                sensor_msgs::msg::PointCloud2 cloud_msg;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                pointcloud_serialization.deserialize_message(&serialized_msg, &cloud_msg);

                // Convert the ROS message to a PCL point cloud
                pcl::PointCloud<PointXYZIRTRAR>::Ptr cloud(new pcl::PointCloud<PointXYZIRTRAR>);
                pcl::fromROSMsg(cloud_msg, *cloud);

                // Clip the point cloud vertically
                pcl::PassThrough<PointXYZIRTRAR> pass;
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(min_z_, max_z_);
                pcl::PointCloud<PointXYZIRTRAR>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZIRTRAR>);
                pass.filter(*cloud_filtered);

                // Convert the filtered cloud back to a ROS message
                sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
                pcl::toROSMsg(*cloud_filtered, filtered_cloud_msg);
                filtered_cloud_msg.header = cloud_msg.header;

                // Serialize the filtered cloud message
                rclcpp::SerializedMessage serialized_filtered_msg;
                pointcloud_serialization.serialize_message(&filtered_cloud_msg, &serialized_filtered_msg);

                // Create a new serialized bag message
                auto serialized_bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
                serialized_bag_msg->topic_name = bag_message->topic_name;
                serialized_bag_msg->time_stamp = bag_message->time_stamp;
                serialized_bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
                *serialized_bag_msg->serialized_data = serialized_filtered_msg.release_rcl_serialized_message();

                // Write the serialized message to the output bag
                bag_writer_.write(serialized_bag_msg);

                // Release memory
                rcutils_uint8_array_fini(serialized_bag_msg->serialized_data.get());
            }
            else if (bag_message->topic_name == "/ouster/imu")
            {
                // Deserialize the message into an Imu message
                sensor_msgs::msg::Imu imu_msg;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                imu_serialization.deserialize_message(&serialized_msg, &imu_msg);

                // Serialize the IMU message
                rclcpp::SerializedMessage serialized_imu_msg;
                imu_serialization.serialize_message(&imu_msg, &serialized_imu_msg);

                // Create a new serialized bag message
                auto serialized_bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
                serialized_bag_msg->topic_name = bag_message->topic_name;
                serialized_bag_msg->time_stamp = bag_message->time_stamp;
                serialized_bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
                *serialized_bag_msg->serialized_data = serialized_imu_msg.release_rcl_serialized_message();

                // Write the serialized message to the output bag
                bag_writer_.write(serialized_bag_msg);

                // Release memory
                rcutils_uint8_array_fini(serialized_bag_msg->serialized_data.get());
            }
        }

        // Close the bags
        bag_reader_.close();
        bag_writer_.close();
    }

    std::string input_bag_path_;
    std::string output_bag_path_;
    double min_z_;
    double max_z_;
    rosbag2_cpp::Reader bag_reader_;
    rosbag2_cpp::Writer bag_writer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarClipperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}