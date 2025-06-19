    #include <ros/ros.h>
    #include <sensor_msgs/PointCloud2.h>
    #include <pcl_conversions/pcl_conversions.h>
    #include <pcl/point_types.h>
    #include <pcl/point_cloud.h>

    // Global publisher object
    ros::Publisher pub;
    ros::Time last_pub_time;

    /**
     * @brief Callback function for incoming PointCloud2 messages.
     * Converts an RGBD point cloud (PointCloud2) to a PCL PointXYZRGB cloud,
     * filters out invalid points, converts it to a PCL PointXYZ cloud,
     * and publishes it as a PointCloud2 message.
     * @param input Pointer to the input PointCloud2 message (RGBD).
     */
    void callback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        ros::Time now = ros::Time::now();
        if ((now - last_pub_time).toSec() < (1.0 / 15.0)) {
            return; // Skip this message
        }
        last_pub_time = now;
        // Convert ROS PointCloud2 to PCL PointXYZRGB cloud
        pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
        pcl::fromROSMsg(*input, cloud_rgb);

        // Create a new PCL PointXYZ cloud
        pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
        cloud_xyz.header = cloud_rgb.header; // Copy header information (e.g., frame_id, stamp)
        cloud_xyz.points.reserve(cloud_rgb.points.size()); // Pre-allocate memory for efficiency

        // Iterate through the RGBD points and add only finite (valid) XYZ points
        for (const auto& pt : cloud_rgb.points)
        {
            // Check if the point's coordinates are finite (not NaN or Inf)
            if (pcl::isFinite(pt))
            {
                // Add the XYZ coordinates to the new cloud
                cloud_xyz.points.emplace_back(pt.x, pt.y, pt.z);
            }
        }

        // Convert the PCL PointXYZ cloud back to a ROS PointCloud2 message
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud_xyz, output);

        // Publish the processed PointCloud2 message
        pub.publish(output);
    }

    /**
     * @brief Main function of the ROS node.
     * Initializes ROS, creates a node handle, sets up a subscriber and a publisher,
     * and enters the ROS event loop.
     * @param argc Argument count.
     * @param argv Argument vector.
     * @return Exit code.
     */
    int main(int argc, char** argv)
    {
        // Initialize ROS node with the name "cloud_xyz_filter"
        // This name will appear in `rosnode list`
        ros::init(argc, argv, "cloud_xyz_filter");
        ros::NodeHandle nh; // Create a ROS node handle

        // Subscribe to the input point cloud topic
        // "/camera/depth_registered/points" is a common topic for RGBD cameras
        // Queue size of 1 means only the latest message is kept if processing is slow
        ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, callback);

        // Advertise the output point cloud topic
        // "/points_xyz" will contain the filtered XYZ point cloud
        // Queue size of 1 means only the latest message is kept for publishers
        pub = nh.advertise<sensor_msgs::PointCloud2>("/points_xyz", 1);

        // Enter the ROS event loop, allowing callbacks to be processed
        ros::spin();

        return 0;
    }
    




void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{


    // ... same as before ...
}