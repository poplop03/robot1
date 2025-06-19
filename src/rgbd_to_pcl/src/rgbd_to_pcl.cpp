#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Global publisher and timer
ros::Publisher pub;
sensor_msgs::PointCloud2 latest_msg;
ros::Time last_pub_time;
double publish_interval = 1.0 / 15.0;  // target: 15 Hz

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    ros::Time now = ros::Time::now();
    if ((now - last_pub_time).toSec() < publish_interval) {
        return; // Skip this frame
    }
    last_pub_time = now;

    // Convert to PCL PointXYZRGB
    pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
    pcl::fromROSMsg(*input, cloud_rgb);

    // Convert to PCL PointXYZ
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    cloud_xyz.header = cloud_rgb.header;
    cloud_xyz.points.reserve(cloud_rgb.points.size());

    for (const auto& pt : cloud_rgb.points) {
        if (pcl::isFinite(pt))
            cloud_xyz.points.emplace_back(pt.x, pt.y, pt.z);
    }

    // Convert to ROS PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_xyz, output);
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_xyz_filter");
    ros::NodeHandle nh("~");

    // Optional: get target publish rate from parameter
    double rate_hz;
    nh.param("publish_rate", rate_hz, 15.0);
    publish_interval = 1.0 / rate_hz;

    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/points_xyz", 1);

    ros::spin();
    return 0;
}
