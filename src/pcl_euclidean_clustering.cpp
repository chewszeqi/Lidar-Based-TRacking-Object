#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <lidar_based_tracking_object/SegmentedClustersArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/lexical_cast.hpp>
#include <pcl/io/pcd_io.h>
#include <iostream>

void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions) {
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);

  pose->position.x = (max_pt.x() + min_pt.x()) / 2;
  pose->position.y = (max_pt.y() + min_pt.y()) / 2;
  pose->position.z = (max_pt.z() + min_pt.z()) / 2;
  pose->orientation.w = 1;

  dimensions->x = max_pt.x() - min_pt.x();
  dimensions->y = max_pt.y() - min_pt.y();
  dimensions->z = max_pt.z() - min_pt.z();

  ROS_INFO("%f %f %f %f %f %f", pose->position.x, pose->position.y, pose->position.z, dimensions->x, dimensions->y, dimensions->z);
}

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("pcl_segmented", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_cluster1", 1);
        pcl_markerpub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    }



    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(input, cloud);

        // Create the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud.makeShared());

        // create the extraction object for the clusters
        std::vector<pcl::PointIndices> cluster;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        // specify euclidean cluster parameters
        ec.setClusterTolerance (1); 
        ec.setMinClusterSize (10);
        ec.setMaxClusterSize (50);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud.makeShared());
        // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
        ec.extract (cluster);
        
        pcl::PCLPointCloud2 outputPCL;
        sensor_msgs::PointCloud2 output;
        int cluster_i=0;
        // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster.begin (); it != cluster.end (); ++it)
        {
            // create a pcl object to hold the extracted cluster
            pcl::PointCloud<pcl::PointXYZ> *cluster = new pcl::PointCloud<pcl::PointXYZ>;
            pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPtr (cluster);

            // now we are in a vector of indices pertaining to a single cluster.
            // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            
                clusterPtr->points.push_back(cloud.makeShared()->points[*pit]);

            cluster_i++;
            visualization_msgs::Marker object_marker;
            object_marker.ns = "objects";
            object_marker.id = cluster_i;
            ROS_INFO("object marker id : %d", object_marker.id);
            object_marker.header.frame_id = "odom";
            object_marker.type = visualization_msgs::Marker::CUBE;
            GetAxisAlignedBoundingBox(clusterPtr, &object_marker.pose, &object_marker.scale);
            object_marker.color.g = 1;
            object_marker.color.a = 0.3;
            pcl_markerpub.publish(object_marker);
            ROS_INFO("%d", cluster_i);

            // object_marker.pose = pose;
            // object_marker.scale = dimensions;

            pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);
            //Convert to ROS data type
            pcl_conversions::fromPCL(outputPCL, output);
            output.header.frame_id = "odom";
            pcl_pub.publish(output);

        }
    }
        
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
    ros::Publisher pcl_markerpub;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_euclidean_clustering");

    cloudHandler handler;

    ros::spin();

    return 0;
}

