#define PCL_NO_PRECOMPILE
#include <pcl/filters/impl/passthrough.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <filesystem>
#include <pcl/point_cloud.h>

// Define the custom point type
struct EIGEN_ALIGN16 PointXYZIRTRAR
{
    PCL_ADD_POINT4D;       // XYZ coordinates (float32, datatype 7)
    float intensity;       // Intensity (float32, datatype 7)
    uint32_t t;            // Timestamp (uint32, datatype 6)
    uint16_t reflectivity; // Reflectivity (uint16, datatype 4)
    uint16_t ring;         // Laser ring number (uint16, datatype 4)
    // uint16_t ambient;      // Ambient light level (uint16, datatype 4)
    // uint32_t range;        // Range (float32, datatype 6)

    // Ensure proper memory alignment for Eigen and SSE
    PCL_MAKE_ALIGNED_OPERATOR_NEW
}; // Ensure 16-byte alignment

// Register the custom point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRTRAR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(uint16_t, ring, ring)); //(uint16_t, ambient, ambient);//(std::uint32_t, range, range));

class PCDPublisher : public rclcpp::Node
{
public:
    PCDPublisher() : Node("pcd_publisher"), map_voxel_size_(0.25)
    {
        this->declare_parameter<std::string>("pcd_file_path", "");
        this->get_parameter("pcd_file_path", pcd_file_path_);

        if (pcd_file_path_.empty() || !std::filesystem::exists(pcd_file_path_))
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid or missing PCD file path: %s", pcd_file_path_.c_str());
            return;
        }

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map1", 10);
        load_and_publish_pcd();
    }

private:
    void load_and_publish_pcd()
    {
        pcl::PointCloud<PointXYZIRTRAR>::Ptr cloud(new pcl::PointCloud<PointXYZIRTRAR>);
        if (pcl::io::loadPCDFile<PointXYZIRTRAR>(pcd_file_path_, *cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", pcd_file_path_.c_str());
            return;
        }

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        // pcl::VoxelGrid<pcl::PointXYZI> sor;
        // sor.setInputCloud(cloud);
        // sor.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);
        // sor.filter(*filtered_cloud);

        // Clip the point cloud vertically
        pcl::PassThrough<PointXYZIRTRAR> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-4, 0.5);
        pcl::PointCloud<PointXYZIRTRAR>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZIRTRAR>);
        pass.filter(*cloud_filtered);

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_filtered, cloud_msg);
        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = this->now();

        RCLCPP_INFO(this->get_logger(), "Publishing PCD file with %ld points", cloud_filtered->points.size());
        // RCLCPP_INFO(this->get_logger(), "Publishing PCD file with %ld points", cloud->points.size());
        pub_->publish(cloud_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    std::string pcd_file_path_;
    double map_voxel_size_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDPublisher>());
    rclcpp::shutdown();
    return 0;
}
