#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert to PCL format
    pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
    pcl::fromROSMsg(*input, cloud_rgb);

    // Convert to PointXYZ only
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    cloud_xyz.header = cloud_rgb.header;
    cloud_xyz.points.reserve(cloud_rgb.points.size());

    for (const auto& pt : cloud_rgb.points)
    {
        if (pcl::isFinite(pt)) {
            cloud_xyz.points.emplace_back(pt.x, pt.y, pt.z);
        }
    }

    // Convert back to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_xyz, output);
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_xyz_filter");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/points_xyz", 1);

    ros::spin();
    return 0;
}
