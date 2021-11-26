#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("pcl_downsampled", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_passthrough", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2 outputPCL;

        pcl::fromROSMsg(input, cloud);

         // create a pcl object to hold the passthrough filtered results
        pcl::PointCloud<pcl::PointXYZ> *cloud_passthrough = new pcl::PointCloud<pcl::PointXYZ>;
        pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtrPassthrough (cloud_passthrough);

        // Create the filtering object
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud.makeShared());
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.00001, 2.5);
        pass.setFilterLimitsNegative (false);
        pass.filter (*CloudPtrPassthrough);

        //convert to pcl::PCLPointCloud2
        pcl::toPCLPointCloud2( *CloudPtrPassthrough ,outputPCL);

        

        //Convert to ROS data type
        pcl_conversions::fromPCL(outputPCL, output);
        pcl_pub.publish(output);

    }
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_downsampling");

    cloudHandler handler;

    ros::spin();

    return 0;
}
