#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_types.h"
#include "pcl/conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.h"

class TableFind : public rclcpp::Node
{
public:
    /// @brief Create a ROS Node that extracts a table (or other flat surface) from a point cloud.
    ///
    /// Subscriptions
    ///    -------------
    ///   pcl_handler (sensor_msgs/msg/PointCloud2): The point cloud to publish
    ///
    /// Publishers
    /// ----------
    /// pcl_cropped (sensor_msgs/msg/PointCloud2): The cropped point cloud
    /// pcl_voxel   (sensor_msgs/msg/PointCloud2): The voxelized point cloud
    /// pcl_inplane (sensor_msgs/msg/PointCloud2): The plane extracted from the point cloud
    TableFind():
        Node("table_find")
    {
        cropped_pub = create_publisher<sensor_msgs::msg::PointCloud2>("pcl_cropped", 10);
        voxel_pub = create_publisher<sensor_msgs::msg::PointCloud2>("pcl_voxel", 10);
        inplane_pub = create_publisher<sensor_msgs::msg::PointCloud2>("pcl_inplane", 10);
        subscriber = create_subscription<sensor_msgs::msg::PointCloud2>
            ("pcl_handler", 10, [this](const sensor_msgs::msg::PointCloud2 & pointcloud_msg)
            {
                // convert ROS 2 message to a PointCloud used by PCL
                auto pcl_in = std::make_shared<pcl::PCLPointCloud2>();
                pcl_conversions::toPCL(pointcloud_msg, *pcl_in);

                // Setup a CropBox filter to crop the point cloude
                pcl::CropBox<pcl::PCLPointCloud2> crop_box{};
                /// minimum corner to crop, in homogeneous coordinates
                crop_box.setMin({-0.75, -0.6, 0.1, 1.0});
                // maximum corner to crob p in homogeneous coordinates
                crop_box.setMax({0.5, 0.1, 2.0, 1.0});

                // Provide the input point cloud
                crop_box.setInputCloud(pcl_in);

                // apply the crop box filter to the output pointcloud
                auto pcl_cropped = std::make_shared<pcl::PCLPointCloud2>();
                crop_box.filter(*pcl_cropped);

                // Convert output point cloud to a ros message and publish
                sensor_msgs::msg::PointCloud2 cropped_msg;
                pcl_conversions::fromPCL(*pcl_cropped, cropped_msg);
                cropped_pub->publish(cropped_msg);


                // Downsample the point cloud to create a less dense point cloud which
                // decreases processing time and puts points on a regular grid
                // Not necessary but it does help performance
                pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
                // parameters for the voxel filter: voxels are 0.01m x 0.01m x 0.01m
                voxel_filter.setLeafSize(0.01, 0.01, 0.01);
                voxel_filter.setInputCloud(pcl_cropped);

                // apply the voxel filter
                auto pcl_voxel = std::make_shared<pcl::PCLPointCloud2>();
                voxel_filter.filter(*pcl_voxel);

                // convert to a ROS 2 message and publish it
                sensor_msgs::msg::PointCloud2 voxel_msg;
                pcl_conversions::fromPCL(*pcl_voxel, voxel_msg);
                voxel_pub->publish(voxel_msg);

                // segment the table from the objects.
                // The segmenter does not accept PCLPointCloud2 objects
                // So we need to convert first
                auto pcl_voxel_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                pcl::fromPCLPointCloud2(*pcl_voxel, *pcl_voxel_xyz);
                pcl::SACSegmentation<pcl::PointXYZ> segmenter;
                segmenter.setModelType(pcl::SACMODEL_PLANE);
                segmenter.setDistanceThreshold(0.02);

                segmenter.setInputCloud(pcl_voxel_xyz);

                auto inliers = std::make_shared<pcl::PointIndices>(); // will hold indices of the points that are in the plane
                pcl::ModelCoefficients coefficients; // holds coefficients for the equation of the plane
                segmenter.segment(*inliers, coefficients);
                if(inliers->indices.size() == 0)
                {
                    // Not able to segment
                    return;
                }

                // We now have indices of the inliers of the plane, which can be used to extract the points
                pcl::ExtractIndices<pcl::PCLPointCloud2> extracter;
                extracter.setInputCloud(pcl_voxel);
                extracter.setIndices(inliers);
                auto pcl_inplane = std::make_shared<pcl::PCLPointCloud2>();
                extracter.filter(*pcl_inplane);

                // convert to ROS 2 message
                sensor_msgs::msg::PointCloud2 inplane_msg;
                pcl_conversions::fromPCL(*pcl_inplane, inplane_msg);
                inplane_pub->publish(inplane_msg);
            });
    }
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inplane_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TableFind>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
