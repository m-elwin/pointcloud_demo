/*import pcl
import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2
from sensor_msgs.msg import PointCloud2
import numpy as np


class TableFind(Node):
    def __init__(self):
        """
        Create a ROS Node that extracts a table (or other flat surface) from a point cloud.

        Subscriptions
        -------------
        pcl_handler (sensor_msgs/msg/PointCloud2): The point cloud to publish

        Publishers
        ----------
        pcl_cropped (sensor_msgs/msg/PointCloud2): The cropped point cloud
        pcl_voxel   (sensor_msgs/msg/PointCloud2): The voxelized point cloud
        pcl_inplane (sensor_msgs/msg/PointCloud2): The plane extracted from the point cloud
        """
        super().__init__("table_find")
        self._sub = self.create_subscription(PointCloud2, "pcl_handler", self.pcl_handler, 10)
        self._cropped = self.create_publisher(PointCloud2, "pcl_cropped", 10)
        self._voxel = self.create_publisher(PointCloud2, "pcl_voxel", 10)
        self._voxel = self.create_publisher(PointCloud2, "pcl_voxel", 10)
        self._inplane = self.create_publisher(PointCloud2, "pcl_inplane", 10)

    def pcl_handler(self, pcl_msg: PointCloud2):

*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_types.h"
#include "pcl/filters/crop_box.h"
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
        subscriber = create_subscription<sensor_msgs::msg::PointCloud2>
            ("pcl_handler", 10, [this](const sensor_msgs::msg::PointCloud2 & pointcloud_msg)
            {
                // convert ROS 2 message to a PointCloud used by PCL
                auto pcl_in = std::make_shared<pcl::PCLPointCloud2>();
                pcl_conversions::toPCL(pointcloud_msg, *pcl_in);

                pcl::CropBox<pcl::PCLPointCloud2> crop_box{};
                /// minimum corner to crop, in homogeneous coordinates
                crop_box.setMin({-0.75, -0.6, 0.1, 1.0});
                // maximum corner to crob p in homogeneous coordinates
                crop_box.setMax({0.5, 0.1, 2.0, 1.0});
                crop_box.setInputCloud(pcl_in);

                auto pcl_cropped = std::make_shared<pcl::PCLPointCloud2>();
                // apply the crop box filter
                crop_box.filter(*pcl_cropped);

                // Convert output point cloud to a ros messages
                sensor_msgs::msg::PointCloud2 cropped_msg;
                pcl_conversions::fromPCL(*pcl_cropped, cropped_msg);

                //  self._cropped.publish(cropped_msg)
            /*
        # Downsample the point cloud to create a less dense point cloud which
        # decreases processing time and puts points on a regular grid
        # Not necessary but it does help performance
        voxel_filter = pcl_cropped.make_voxel_grid_filter()
        voxel_filter.set_leaf_size(0.01, 0.01, 0.01)
        pcl_voxel = voxel_filter.filter()

        voxel_msg = sensor_msgs_py.point_cloud2.create_cloud_xyz32(
            pcl_msg.header,
            pcl_voxel.to_array()
        )

        # publish the voxelized point cloud
        self._voxel.publish(voxel_msg)

        # segment the table from the objects
        segmenter = pcl_voxel.make_segmenter()
        segmenter.set_model_type(pcl.SACMODEL_PLANE)
        segmenter.set_distance_threshold(0.02)
        indices, coefficients = segmenter.segment()

        # We now have indices of the inliers of the plane
        # and the coefficients of that plane
        # Next, need to convert to point clouds
        if len(indices) == 0:
            # No plane was found
            return

        # Get all the points that lie in the plane and create a new pointcloud with them
        pcl_inplane = np.copy(pcl_voxel)[indices]
        inplane_msg = sensor_msgs_py.point_cloud2.create_cloud_xyz32(
            pcl_msg.header,
            pcl_inplane
        )

        self._inplane.publish(inplane_msg)*/
            });
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber;

};

    /*        self._sub = self.create_subscription(PointCloud2, "pcl_handler", self.pcl_handler, 10)
        self._cropped = self.create_publisher(PointCloud2, "pcl_cropped", 10)
        self._voxel = self.create_publisher(PointCloud2, "pcl_voxel", 10)
        self._voxel = self.create_publisher(PointCloud2, "pcl_voxel", 10)
        self._inplane = self.create_publisher(PointCloud2, "pcl_inplane", 10)
    */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TableFind>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
