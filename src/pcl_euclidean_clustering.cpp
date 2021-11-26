#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("pcl_segmented", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_cluster1", 1);
        pcl_clusterpub = nh.advertise<lidar_based_tracking_object::SegmentedClustersArray>("pcl_cluster2", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        enum COLOR_RGB{
            RED=0,
            GREEN,
            BLUE,
            COLOR_MAX
        };
        const int CLUSTER_MAX = 10;
        const int CLUSTER_COLOR[CLUSTER_MAX][COLOR_MAX] = {
            {230, 0, 18},{243, 152, 18}, {255, 251, 0},
            {143, 195, 31},{0, 153, 68}, {0, 158, 150},
            {0, 160, 233},{0, 104, 183}, {29, 32, 136},
            {146, 7, 131}
        };
        pcl::PointCloud<pcl::PointXYZ> cloud;
        // pcl::PointCloud<pcl::PointXYZ> cloud_clustered;
        //pcl::PointCloud<pcl::PointXYZ> cluster;

        pcl::fromROSMsg(input, cloud);

        // Create the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud.makeShared());

        // create the extraction object for the clusters
        std::vector<pcl::PointIndices> cluster;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        // specify euclidean cluster parameters
        ec.setClusterTolerance (5.0); // 2cm
        ec.setMinClusterSize (0.5);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud.makeShared());
        // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
        ec.extract (cluster);

        
        pcl::PCLPointCloud2 outputPCL;
        lidar_based_tracking_object::SegmentedClustersArray CloudClusters;
        sensor_msgs::PointCloud2 output;
        int cluster_i=0;
        // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster.begin (); it != cluster.end (); ++it)
        {

            // create a new clusterData message object
            //obj_recognition::ClusterData clusterData;


            // create a pcl object to hold the extracted cluster
            pcl::PointCloud<pcl::PointXYZ> *cluster = new pcl::PointCloud<pcl::PointXYZ>;
            pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPtr (cluster);

            // now we are in a vector of indices pertaining to a single cluster.
            // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            {
                // clusterPtr->points[*pit].r = CLUSTER_COLOR[cluster_i][RED];
                // clusterPtr->points[*pit].g = CLUSTER_COLOR[cluster_i][GREEN];
                // clusterPtr->points[*pit].b = CLUSTER_COLOR[cluster_i][BLUE];
                clusterPtr->points.push_back(cloud.makeShared()->points[*pit]);

                }

            // cluster_i++;
            // if(cluster_i >= CLUSTER_MAX){
            //     break;

            //convert to pcl::PCLPointCloud2
            pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);

            

            //Convert to ROS data type
            pcl_conversions::fromPCL(outputPCL, output);

            

            // add the cluster to the array message
            //clusterData.cluster = output;
            CloudClusters.clusters.push_back(output);
            output.header.frame_id = "odom";
            pcl_pub.publish(output);
        }
        
        pcl_clusterpub.publish(CloudClusters);
    }
        
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
    ros::Publisher pcl_clusterpub;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_euclidean_clustering");

    cloudHandler handler;

    ros::spin();

    return 0;
}

