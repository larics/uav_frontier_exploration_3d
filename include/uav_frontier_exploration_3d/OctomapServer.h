#ifndef OCTOMAP_H_
#define OCTOMAP_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeLUT.h>

#include <stdio.h>
#include <fstream>
#include <stdlib.h>
#include <math.h>

#include "yaml-cpp/yaml.h"

using namespace std;
using namespace octomap;
using octomap_msgs::Octomap;

// Generates octomap and finds changed cells
namespace octomap_server
{
  class OctomapServer
  {
    public:
      typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
      typedef pcl::PointCloud<pcl::PointXYZI> PCLPointCloudI;

      OctomapServer();
      virtual ~OctomapServer();

      void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
      void globalPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
      bool configureFromFile(string config_filename);
      void runDefault();
      octomap::OcTree* getOcTree(){ return m_octree; }
      PCLPointCloudI getChangedCells() { return m_changedCells; }
      geometry_msgs::Pose getCurrentUAVPosition() { return m_uavCurrentPose; }
      void genNeighborCoord(float x, float y, float z);
      void genNeighborCoord(
        octomap::OcTreeKey start_key, std::vector<octomap::point3d>& occupiedNeighbor);
      void publishVolume();

    protected:
      inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min)
      {
        for (unsigned i = 0; i < 3; ++i)
          min[i] = std::min(in[i], min[i]);
      };

      inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max)
      {
        for (unsigned i = 0; i < 3; ++i)
          max[i] = std::max(in[i], max[i]);
      };

      virtual void insertScan(const PCLPointCloud& cloud);
      bool isSpeckleNode(const octomap::OcTreeKey& key) const;
      void trackChanges(PCLPointCloudI& changedCells);
      void transformAndFilterPointCloud(tf::StampedTransform& sensorToWorldTf,
        PCLPointCloud& pc);
      void publishOccAndFree();
      void publishBinaryOctoMap();
      bool saveOctomapServiceCb(
        std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

      ros::NodeHandle m_nh;
      ros::Publisher  m_markerOccPub, m_markerFreePub, m_binaryMapPub, m_pubVolumes;
      ros::Subscriber m_pointCloudSub, m_uavGlobalPoseSub;
      ros::ServiceServer m_saveOctomapServer;
      tf::TransformListener m_tfListener;
      sensor_msgs::PointCloud2 m_currPointCloud;

      octomap::OcTree* m_octree {NULL};
      octomap::KeyRay m_keyRay;  // temp storage for ray casting
      octomap::KeyRay m_keyRaysphere;
      octomap::OcTreeKey m_updateBBXMin;
      octomap::OcTreeKey m_updateBBXMax;
      octomap::OcTreeLUT m_lut {16};

      PCLPointCloudI m_changedCells;
      std::string m_worldFrameId, m_baseFrameId; 

      unsigned m_treeDepth, m_maxTreeDepth;
      double m_resolution, m_probHit, m_probMiss, m_thresMin, m_thresMax, m_maxRange,
        m_totalVol;

      double m_pointcloudMinZ {-std::numeric_limits<double>::max()},
        m_pointcloudMaxZ {std::numeric_limits<double>::max()},
        m_occupancyMinZ {-std::numeric_limits<double>::max()},
        m_occupancyMaxZ {std::numeric_limits<double>::max()};

      bool m_filterSpeckles {false},
        m_compressMap {true},
        m_pointCloudReceivedFlag {false};

      string m_configFilename;
      ofstream m_logfile;
      geometry_msgs::Pose m_uavCurrentPose; 
      string m_filename, m_filePath;  
    };
}
#endif