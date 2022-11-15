import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2
from sensor_msgs.msg import PointCloud2

class TableFind(Node):
    def __init__(self):
        """
        Subscriptions
        -------------
        pcl_handler (sensor_msgs/msg/PointCloud2): The point cloud to publish

        Publishers
        ----------
        pcl_cropped (sensor_msgs/msg/PointCloud2): The cropped point cloud
        """
        super().__init__("table_find")
        self._sub = self.create_subscription(PointCloud2, "pcl_handler", self.pcl_handler, 10)
        self._cropped = self.create_publisher(PointCloud2, "pcl_cropped", 10)

    def pcl_handler(pointcloud):
        """ Get the point cloud and perform some transformations and publish them """
        # Convert ROS2 message to a PointCloud used by PCL
        points = pcl.PointCloud(sensor_msgs_py.point_cloud2.read_points(pointcloud))

        # Apply the CropBox filter to remove points that are outside of a given bounding box
        # See https://github.com/strawlab/python-pcl/blob/master/examples/cropbox.py for another example
        crop_out = PointCloud()
        crop_box = points.make_cropbox()
        crop_box.set_MinMax(-0.75, -0.6, 0.1, 0.5, 0.1, 2.0)
        crop_box.Filtering(crop_out)

        # Convert output point cloud to a ros message
        ros_cropped = sensor_msgs_py.point_cloud2.create_cloud_xyz32(
            points.header,
            crop_out.to_array()
        )

        self._cropped.publish(ros_cropped)


def table_entry(args=None):
    rclpy.init(args=args)
    node = TableFind()
    rclpy.spin(node)
    rclpy.shutdown()
