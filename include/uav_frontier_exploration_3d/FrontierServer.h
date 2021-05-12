#ifndef FRONTIER_EXPLORATION_H_
#define FRONTIER_EXPLORATION_H_

#include <uav_frontier_exploration_3d/OctomapServer.h>
#include <uav_frontier_exploration_3d/BestFrontier.h>
//Mean shift clustering
#include <uav_frontier_exploration_3d/ClusteringAlgorithm.h>
#include <dlib/clustering.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>

namespace frontier_server
{
  enum ExplorationState {
    OFF,
    CHECKFORFRONTIERS,
    ON,
    POINTREACHED
  };

  inline const char* ToString(ExplorationState v)
  {
    switch (v)
    {
      case OFF:               return "OFF - Waiting for servis call";
      case CHECKFORFRONTIERS: return "CHECKFORFRONTIERS - Waiting for frontiers";
      case ON:                return "ON - calculate best frontier";
      case POINTREACHED:      return "POINTREACHED - Current goal point is reached";
      default:                return "[Unknown ExplorationState]";
    }
  }

  class FrontierServer
  {
    public:
      typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
      typedef pcl::PointCloud<pcl::PointXYZI> PCLPointCloudI;

      FrontierServer();
      virtual ~FrontierServer();

      bool configureFromFile(string config_filename);
      void setStateAndPublish(ExplorationState state);
      void run();

    protected:
      KeySet findFrontier(PCLPointCloudI& changedCells);
      void searchForParentsAndPublish();
      void updateGlobalFrontier(KeySet& globalFrontierCell);
      void clusterFrontierAndPublish();
      void pointReachedCallback(std_msgs::Bool msg);
      void currentReferenceCallback(geometry_msgs::PoseStamped msg);
      void checkClusteredCells();
      bool isPointAlreadyAssigned(point3d point);
    
      void keyToPointVector(KeySet& frontierCells, 
        vector<geometry_msgs::Point>& originalPointsVector);
      void pointVectorToKey(vector<geometry_msgs::Point>& points,
        vector<OcTreeKey>& clusterCellsKey);
      void setPointAsInvalid(point3d point);
      void publishParentFrontier();
      void publishClusteredFrontier();
      void publishBestFrontier();
      void publishUAVGoal(point3d goal);
      bool toggleExplorationServiceCb(std_srvs::SetBool::Request& request, 
			  std_srvs::SetBool::Response& response)
      {
        m_explorationToggled = request.data;
        if (m_explorationToggled) 
          std::cout << "Exploration ON." << std::endl << std::endl;
        else 
          std::cout << "Exploration OFF." << std::endl << std::endl;
        response.success = true;
	      response.message = "toggleExplorationService called!";
        return true;
      }

      ros::NodeHandle m_nh;
      ros::Publisher m_markerFrontierPub, m_markerClusteredFrontierPub,
        m_bestFrontierPub, m_markerCandidatesPub,m_frontierMapPub, m_uavGoalPub,
        m_pubEsmState;
      ros::Subscriber m_pointReachedSub, m_currentReferenceSub;

      octomap::OcTree* m_octree {NULL};
      octomap_server::OctomapServer m_octomapServer;

      std::string m_worldFrameId, m_baseFrameId; 

      unsigned m_treeDepth, m_explorationDepth;
      double m_rate {1}, m_resolution, m_explorationRadius, m_explorationMinX,
        m_explorationMaxX, m_explorationMinY, m_explorationMaxY,m_explorationMinZ,
        m_explorationMaxZ, m_kernelBandwidth;

      bool m_currentGoalReached {true};
      bool m_explorationToggled {false};

      string m_configFilename;
      
      // The newest frontier cells
      KeySet m_globalFrontierCellsUpdated, 
        m_clusteredCells, m_clusteredCellsUpdated, m_parentFrontierCells,
        m_invalidParentCells;
  
      ofstream m_logfile;
      
      best_frontier::BestFrontier m_bestFrontierServer;
      PCLPointCloudI m_changedCells;
      point3d m_bestFrontierPoint;
      vector<point3d> m_allUAVGoals;

      geometry_msgs::Pose m_uavCurrentPose;
      geometry_msgs::PoseStamped m_uavCurrentReference;
      ros::ServiceServer m_serviceExploration;  
      ExplorationState m_currentState = ExplorationState::OFF;
    };
}
#endif