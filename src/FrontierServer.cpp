/*
 * FrontierServer.cpp
 *
 *  Created on: April 15, 2020
 *      Author: Ana Batinovic
 */

#include <uav_frontier_exploration_3d/FrontierServer.h>

namespace frontier_server
{
	FrontierServer::FrontierServer()
	{
		ros::NodeHandle private_nh {ros::NodeHandle("~")};
		private_nh.param("rate", m_rate, m_rate);

		// Read from yaml file
		private_nh.param("exploration_config_filename", m_configFilename, m_configFilename);
		configureFromFile(m_configFilename);

		m_logfile.open("/log_frontier.txt");
		m_logfile << "This is a log file for 3D-Frontier" << endl;

		// Initialize publishers
		m_bestFrontierPub = m_nh.advertise<
			visualization_msgs::Marker>("best_frontier_marker", 1, false);
		m_markerFrontierPub = m_nh.advertise<
			visualization_msgs::MarkerArray>("frontier_cells_vis_array", 1, false);
		m_markerClusteredFrontierPub = m_nh.advertise<
			visualization_msgs::MarkerArray>("clustered_frontier_cells_vis_array", 1, false);
		m_uavGoalPub = m_nh.advertise<
			geometry_msgs::PoseStamped>("exploration/goal", 1, false);
		m_pubEsmState = m_nh.advertise<std_msgs::Int32>("exploration/state", 1);

		// Initialize subscribers
		m_pointReachedSub = m_nh.subscribe("point_reached", 1, 
			&FrontierServer::pointReachedCallback, this);
		m_currentReferenceSub = m_nh.subscribe("carrot/pose", 1, 
			&FrontierServer::currentReferenceCallback, this);

		// Initialize position hold service if exploration is off
		m_serviceExploration = m_nh.advertiseService("exploration/toggle",
			&FrontierServer::toggleExplorationServiceCb, this);
	}

	FrontierServer::~FrontierServer()
	{
		if (m_octree)
		{
			delete m_octree;
			m_octree = NULL;
		}	
	}

	bool FrontierServer::configureFromFile(string config_filename)
	{
		cout << "FrontierServer - Configuring uav exploration from file: " << endl;
		cout << "  " << config_filename << endl;
		
		// Open yaml file with configuration
		YAML::Node config = YAML::LoadFile(config_filename);

		// Get params
		m_worldFrameId = config["exploration"]["global_frame"].as<string>();
		m_baseFrameId = config["exploration"]["base_link_frame"].as<string>();
		m_explorationDepth = config["exploration"]["depth"].as<unsigned>();
		m_explorationRadius = config["exploration"]["radius"].as<double>();
		m_explorationMinX = config["exploration"]["bbx_minX"].as<double>();
		m_explorationMaxX = config["exploration"]["bbx_maxX"].as<double>();
		m_explorationMinY = config["exploration"]["bbx_minY"].as<double>();
		m_explorationMaxY = config["exploration"]["bbx_maxY"].as<double>();
		m_explorationMinZ = config["exploration"]["bbx_minZ"].as<double>();
		m_explorationMaxZ = config["exploration"]["bbx_maxZ"].as<double>();

		m_resolution = config["octomap"]["resolution"].as<double>();
		m_treeDepth = config["octomap"]["octree_depth"].as<unsigned>();

		m_kernelBandwidth = config["clustering"]["kernel_bandwidth"].as<double>();
		return true;
	}

	KeySet FrontierServer::findFrontier(PCLPointCloudI& changedCells)
	{
		ros::WallTime startTime = ros::WallTime::now();
		KeySet globalFrontierCells;
		// m_logfile << "findFrontier" << endl;
		bool unknownCellFlag {false};
		bool freeCellFlag {false};
		std::vector<octomap::point3d> changedCellNeighbor;
		int frontierSize {0};
		for(int i = 0; i < changedCells.points.size(); i++)
		{
			// Check if the point is inside the bounding box
			if(changedCells.points[i].x < m_explorationMinX ||
				changedCells.points[i].x > m_explorationMaxX ||
				changedCells.points[i].y < m_explorationMinY ||
				changedCells.points[i].y > m_explorationMaxY ||
				changedCells.points[i].z < m_explorationMinZ ||
				changedCells.points[i].z > m_explorationMaxZ) continue;
			// Get changed point
			point3d changedCellPoint(
				changedCells.points[i].x,
				changedCells.points[i].y, 
				changedCells.points[i].z);

			// Transform from point to key
			OcTreeKey changedCellKey;
			if (!m_octree->coordToKeyChecked(changedCellPoint, changedCellKey)) 
			{
				OCTOMAP_ERROR_STR("Error in search: [" 
					<< changedCellPoint << "] is out of OcTree bounds!");
				return globalFrontierCells;
			}
			
			// Check point state: free/occupied
			OcTreeNode* changedCellNode = m_octree->search(changedCellKey);
			bool changedCellOccupied = m_octree->isNodeOccupied(changedCellNode);
			if(!changedCellOccupied)
			{
				unknownCellFlag = false;
				freeCellFlag = false;
				m_octomapServer.genNeighborCoord(changedCellKey, changedCellNeighbor);
				for (std::vector<point3d>::iterator iter = changedCellNeighbor.begin();
					iter != changedCellNeighbor.end(); iter++)
				{
					// Check point state: unknown(null)/free
					OcTreeNode* node = m_octree->search(*iter);
					if(node == NULL)
						unknownCellFlag = true;
					else if(!m_octree->isNodeOccupied(node))
						freeCellFlag = true;
				}
				if(unknownCellFlag && freeCellFlag)
					globalFrontierCells.insert(changedCellKey);
					frontierSize++;
			}
		}
	
		m_logfile << "Number of frontiers:" << frontierSize << endl;
		double total_time = (ros::WallTime::now() - startTime).toSec();
		m_logfile << "findFrontier - used total: "<< total_time << " sec" <<endl;
		return globalFrontierCells;
	}

	void FrontierServer::updateGlobalFrontier(KeySet& globalFrontierCells)
	{
		// m_logfile << "updateGlobalFrontier" << endl;
		ros::WallTime startTime = ros::WallTime::now();
		int frontierSize {0};
		m_globalFrontierCellsUpdated.clear();
		for(KeySet::iterator iter = globalFrontierCells.begin(), end = globalFrontierCells.end();
			iter!= end; ++iter)
		{
			frontierSize++;
		}
		m_logfile << "Number of global frontiers before:" << frontierSize << endl;
		bool unknownCellFlag {false};
		bool freeCellFlag {false};
		bool occupiedCellFlag {false};
		int deleted {0};
		// Find in globalFrontier cells that are not frontier anymore --vol2
		// Delete them from globalFrontierCells
		std::vector<octomap::point3d> changedCellNeighbor;
		for(KeySet::iterator cell_iter = globalFrontierCells.begin(), end = globalFrontierCells.end();
			cell_iter!= end; )
		{
			// If current cell if free, check its neighbors
			unknownCellFlag = false;
			occupiedCellFlag = false;
			freeCellFlag = false;
			m_octomapServer.genNeighborCoord(*cell_iter, changedCellNeighbor);
			for (std::vector<point3d>::iterator neighbor_iter = changedCellNeighbor.begin();
				neighbor_iter != changedCellNeighbor.end(); neighbor_iter++)
			{

				// Check for neighbors
				OcTreeNode* node = m_octree->search(*neighbor_iter);
				if(node == NULL)
					unknownCellFlag = true;
				else if(!m_octree->isNodeOccupied(node))
					freeCellFlag = true;
				else if (m_octree->isNodeOccupied(node))
					occupiedCellFlag = true;
			}
			if(!unknownCellFlag || occupiedCellFlag)
			{
				// globalFrontierCells.erase(*cell_iter);
				deleted++;	
			}
			else m_globalFrontierCellsUpdated.insert(*cell_iter);
			
			cell_iter++;				
		}
		m_logfile << "Number of deleted frontiers:" << deleted << endl;
		// Calculate number of frontiers
		frontierSize = 0;
		for(KeySet::iterator iter = m_globalFrontierCellsUpdated.begin(), end = m_globalFrontierCellsUpdated.end();
			iter!= end; ++iter)
		{
			frontierSize++;
		}
		m_logfile << "Number of global frontiers after:" << frontierSize << endl;

		double total_time = (ros::WallTime::now() - startTime).toSec();
		m_logfile << "updateGlobalFrontier - used total: "<< total_time << " sec" <<endl;
	}

	void FrontierServer::searchForParentsAndPublish()
  {
    // m_logfile << "searchForParents" << endl;
		ros::WallTime startTime = ros::WallTime::now();
		// Search in globalFrontierCells after update
    for(KeySet::iterator iter = m_globalFrontierCellsUpdated.begin(), end = m_globalFrontierCellsUpdated.end(); iter != end; ++iter)
    {
      // Search for parent on the desire depth 
      OcTreeNode* parentNodePtr = m_octree->search(*iter, m_explorationDepth);
      parentNodePtr->setValue(-1);
    }
  
    // Iter over desire depth and ask if the node is frontier
		int counter {0};
		m_parentFrontierCells.clear();
    for (OcTree::iterator it = m_octree->begin(m_explorationDepth), end = m_octree->end(); it != end; ++it)
    {
      if(it->getValue() == -1)
      {
				// I found my parent on m_explorationDepth
				// Check if key in m_invalidParentCells
				auto found =  m_invalidParentCells.find(it.getKey());
				if (found != m_invalidParentCells.end())
				{
					continue;
				}
				else
				{				
					m_parentFrontierCells.insert(it.getKey());
					counter++;
				}
      }
    }

		cout << "FrontierServer - parents number: " << counter << endl;
		m_logfile << "number of parents: " << counter << endl;
		double total_time = (ros::WallTime::now() - startTime).toSec();
		m_logfile << "SearchForParents - used total: "<< total_time << " sec" <<endl;
		publishParentFrontier();
  }

	void FrontierServer::clusterFrontierAndPublish()
	{
		// m_logfile << "clusterFrontierAndPublish" << endl;
		ros::WallTime startTime_evaluation = ros::WallTime::now();
		m_clusteredCells.clear();
		
		// Preprocess put the frontier cells into a vector
		std::vector<geometry_msgs::Point> originalPointsVector {};
		std::vector<geometry_msgs::Point> clusteredPointsVector {};
		keyToPointVector(m_parentFrontierCells, originalPointsVector);
		MSCluster *cluster = new MSCluster();
		cluster->getMeanShiftClusters(
			originalPointsVector, clusteredPointsVector, m_kernelBandwidth);
		vector<OcTreeKey> clusterCellsKey {};
		pointVectorToKey(clusteredPointsVector, clusterCellsKey);
		for (std::vector<OcTreeKey>::iterator iter = clusterCellsKey.begin();
			iter != clusterCellsKey.end(); iter++)
		{
			m_clusteredCells.insert(*iter);
		}
		delete cluster;
		m_logfile << "cluster_size: " << m_clusteredCells.size() << endl;
		// cout << "cluster_size: " << m_clusteredCells.size() << endl;
		double total_time_evaluation = (ros::WallTime::now() - startTime_evaluation).toSec();
		m_logfile << "clusterFrontier used total: " << total_time_evaluation << " sec" << endl;
		checkClusteredCells();
		publishClusteredFrontier();
	}

	void FrontierServer::pointReachedCallback(std_msgs::Bool msg)
	{
		if (msg.data)
		{
			// ROS_INFO("Current goal point is reached!");
			m_currentGoalReached = true;
		}
	}

	void FrontierServer::currentReferenceCallback(geometry_msgs::PoseStamped msg)
	{
		m_uavCurrentReference = msg;
	} 

	void FrontierServer::checkClusteredCells()
	{
		int deletedNum {0};
		m_clusteredCellsUpdated.clear();
		for(KeySet::iterator iter = m_clusteredCells.begin(), 
			end = m_clusteredCells.end(); iter != end; ++iter)
		{
			// Get cell position
			point3d tempCellPosition = m_octree->keyToCoord(*iter);		
			if (isPointAlreadyAssigned(tempCellPosition))
			{
				// Remove it from candidates
				// m_clusteredCells.erase(*iter);
				deletedNum++;
			}
			else 
			{
				m_clusteredCellsUpdated.insert(*iter);
			}
		}
		cout << "Delete candidates num: " << deletedNum << endl;
	}

	bool FrontierServer::isPointAlreadyAssigned(point3d point)
	{
		// If point is in m_allUAVGoals return true
		// For each element in m_allUAVGoals check
		for (int i = 0; i < m_allUAVGoals.size(); i++)
		{
			if (
			fabs(m_allUAVGoals[i].x() - point.x()) < m_explorationRadius &&
			fabs(m_allUAVGoals[i].y() - point.y()) < m_explorationRadius &&
			fabs(m_allUAVGoals[i].z() - point.z()) < m_explorationRadius)
			{
				// Point is too close to be assigned 
				ROS_WARN("Similar point has been already assigned!");
				return true;
			}	
		}
		return false;
	}
	
	void FrontierServer::keyToPointVector(KeySet& frontierCells, 
		vector <geometry_msgs::Point>& originalPointsVector)
	{
		for(KeySet::iterator iter = frontierCells.begin(), end = frontierCells.end();
			iter!= end; ++iter)
		{
				OcTreeKey tempCell;
				tempCell = *iter;

				point3d tempCellCoordinates;
				tempCellCoordinates = m_octree->keyToCoord(tempCell);

				geometry_msgs::Point tempCellPoint;
				tempCellPoint.x = tempCellCoordinates.x();
				tempCellPoint.y = tempCellCoordinates.y();
				tempCellPoint.z = tempCellCoordinates.z();

				originalPointsVector.push_back(tempCellPoint);
		}
	}

	void FrontierServer::pointVectorToKey(vector<geometry_msgs::Point>& points,
		vector<OcTreeKey>& clusterCellsKey)
	{
		for (int i = 0; i < points.size(); i++)
		{
			point3d tempCellCoordinates;
			tempCellCoordinates.x() = points[i].x;
			tempCellCoordinates.y() = points[i].y;
			tempCellCoordinates.z() = points[i].z;
			// Transform from point to key
			OcTreeKey tempCellKey;
			if (!m_octree->coordToKeyChecked(tempCellCoordinates, tempCellKey)) 
			{
				OCTOMAP_ERROR_STR("Error in search: [" 
					<< tempCellCoordinates << "] is out of OcTree bounds!");
				return;
			} 
			clusterCellsKey.push_back(tempCellKey);
		}
	}

	void FrontierServer::setPointAsInvalid(point3d point)
	{
		cout << "Setting point: " << point << " as invalid" << endl;
			// Transform from point to key
		OcTreeKey invalidCellKey;
		if (!m_octree->coordToKeyChecked(point, invalidCellKey)) 
		{
			OCTOMAP_ERROR_STR("Error in search: [" 
				<< point << "] is out of OcTree bounds!");
			return;
		}
		m_invalidParentCells.insert(invalidCellKey);
	}

	void FrontierServer::setStateAndPublish(ExplorationState state)
	{
		m_currentState = state;
		// Publish current state
		std_msgs::Int32 stateMsg;
		stateMsg.data = m_currentState;
		m_pubEsmState.publish(stateMsg);
		ROS_INFO_STREAM("FrontierServer::updateStatus - state activated: " << ToString(m_currentState));
	}

	void FrontierServer::run()
	{
		ros::Rate loopRate(m_rate);
		setStateAndPublish(ExplorationState::OFF);

		while (ros::ok())
		{
			ros::WallTime startTime = ros::WallTime::now();
			ros::spinOnce();
			// Initialize before switch
			KeySet globalFrontierCells;
			switch (m_currentState)
			{
				case ExplorationState::OFF:
					if(m_explorationToggled)
						setStateAndPublish(ExplorationState::CHECKFORFRONTIERS);
					break;

				case ExplorationState::CHECKFORFRONTIERS:
					m_octomapServer.runDefault();
					m_octomapServer.publishVolume();
					m_uavCurrentPose = m_octomapServer.getCurrentUAVPosition();
					if(!m_currentGoalReached)
						ROS_WARN_STREAM_THROTTLE(3.0,
						m_bestFrontierPoint.x() << " " << m_bestFrontierPoint.y() << " " 
						<< m_bestFrontierPoint.z() << " -> Goal published!");
					// Update octomap
					m_octree = m_octomapServer.getOcTree();	
					// Get changed cells
					m_changedCells = m_octomapServer.getChangedCells();
					if (m_changedCells.size() > 0)
						setStateAndPublish(ExplorationState::ON);
					break;

				case ExplorationState::ON:
					globalFrontierCells = findFrontier(m_changedCells);
					// Delete frontiers that are explored now
					updateGlobalFrontier(globalFrontierCells);	
					// Find frontiers on the upper level (m_explorationDepth) and publish it
					searchForParentsAndPublish();
					// If there is no parents switch to CHECKFORFRONTIERS
					if (!m_parentFrontierCells.size() > 0) 
						setStateAndPublish(ExplorationState::CHECKFORFRONTIERS);
					// If the previous goal is reached
					else if (m_currentGoalReached)
						setStateAndPublish(ExplorationState::POINTREACHED);
					else 
						setStateAndPublish(ExplorationState::CHECKFORFRONTIERS);
					break;

				case ExplorationState::POINTREACHED:
					// Find Best Frontier
					m_currentGoalReached = false;
					// Delete candidates that are too close to prevoius assigned points
					
					clusterFrontierAndPublish();
					point3d currentPoint3d(m_uavCurrentPose.position.x, 
						m_uavCurrentPose.position.y, m_uavCurrentPose.position.z);
					// Simulation bag
					m_bestFrontierPoint = 
							m_bestFrontierServer.bestFrontierInfGain(m_octree, currentPoint3d, m_clusteredCellsUpdated);
					// m_bestFrontierPoint = 
					// 	m_bestFrontierServer.closestFrontier(m_octree, currentPoint3d, m_clusteredCellsUpdated);
					m_logfile << "Best frontier: " << m_bestFrontierPoint << endl;
					
					m_allUAVGoals.push_back(m_bestFrontierPoint);
					cout << "Best frontier: " << m_bestFrontierPoint << endl;
					publishBestFrontier();
					publishUAVGoal(m_bestFrontierPoint);
					ros::Duration(0.05).sleep();
					setStateAndPublish(ExplorationState::CHECKFORFRONTIERS);
					// }
					break;
			}
			ros::WallTime currentTime = ros::WallTime::now();
			m_logfile << "Frontier exploration used total :" << (currentTime - startTime).toSec() << " sec" << endl;
			loopRate.sleep();
		}
	}

	void FrontierServer::publishParentFrontier()
	{
		m_logfile << "publishFrontier" << endl;
		// init markers for free space:
		visualization_msgs::MarkerArray frontierNodesVis;
		// each array stores all cubes of a different size, one for each depth level:
		frontierNodesVis.markers.resize(m_explorationDepth + 1);
		
		int counter {0};
		for (OcTree::iterator it = m_octree->begin(m_explorationDepth),end = m_octree->end(); it != end; ++it)
		{
			bool isfron = false;
			for(KeySet::iterator iter = m_parentFrontierCells.begin(), end = m_parentFrontierCells.end(); iter!= end; ++iter)
			{
				octomap::point3d fpoint;
				fpoint = m_octree->keyToCoord(*iter);
				
				if (fabs(it.getX() - fpoint.x()) <= m_resolution /2 &&
					fabs(it.getY() - fpoint.y()) <= m_resolution /2 &&
					fabs(it.getZ() - fpoint.z()) <= m_resolution /2)
				{
					isfron = true;
					counter++;
				}
					
			}
			if (isfron)
			{
				double x = it.getX();
				double y = it.getY();
				double z = it.getZ();
	
				unsigned idx = it.getDepth();
				assert(idx < frontierNodesVis.markers.size());

				geometry_msgs::Point cubeCenter;
				cubeCenter.x = x;
				cubeCenter.y = y;
				cubeCenter.z = z;

				frontierNodesVis.markers[idx].points.push_back(cubeCenter);
			} 
		}
		// finish MarkerArray:
		std_msgs::ColorRGBA colorFrontier;
		colorFrontier.r = 1.0;
		colorFrontier.g = 0.0;
		colorFrontier.b = 0.0;
		colorFrontier.a = 1.0;
		for (unsigned i= 0; i < frontierNodesVis.markers.size(); ++i)
		{
			double size = m_octree->getNodeSize(i);

			frontierNodesVis.markers[i].header.frame_id = m_worldFrameId;
			frontierNodesVis.markers[i].header.stamp = ros::Time::now();
			frontierNodesVis.markers[i].ns = "red";
			frontierNodesVis.markers[i].id = i;
			frontierNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
			frontierNodesVis.markers[i].scale.x = size;
			frontierNodesVis.markers[i].scale.y = size;
			frontierNodesVis.markers[i].scale.z = size;
			frontierNodesVis.markers[i].color = colorFrontier;

			if (frontierNodesVis.markers[i].points.size() > 0)
				frontierNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
			else
				frontierNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
		}
		m_markerFrontierPub.publish(frontierNodesVis);
	}

	void FrontierServer::publishClusteredFrontier()
	{
		m_logfile << "publishClusteredFrontier" << endl;
		// init markers for free space:
		visualization_msgs::MarkerArray frontierNodesVis;
		// each array stores all cubes of a different size, one for each depth level:
		frontierNodesVis.markers.resize(m_explorationDepth + 1);

		for (OcTree::iterator it = m_octree->begin(m_explorationDepth), end = m_octree->end(); it != end; ++it)
		{
			bool isfron = false;
			for(KeySet::iterator iter = m_clusteredCellsUpdated.begin(), end = m_clusteredCellsUpdated.end(); iter!= end; ++iter)
			{
				octomap::point3d fpoint;
				fpoint = m_octree->keyToCoord(*iter);
				if (fabs(it.getX() - fpoint.x()) <= m_resolution /2 &&
					fabs(it.getY() - fpoint.y()) <= m_resolution /2 &&
					fabs(it.getZ() - fpoint.z()) <= m_resolution /2) isfron = true;
			}
			if (isfron)
			{
				double x = it.getX();
				double y = it.getY();
				double z = it.getZ();
	
				unsigned idx = it.getDepth();
				assert(idx < frontierNodesVis.markers.size());

				geometry_msgs::Point cubeCenter;
				cubeCenter.x = x;
				cubeCenter.y = y;
				cubeCenter.z = z;

				frontierNodesVis.markers[idx].points.push_back(cubeCenter);
			} 
		}
		// finish MarkerArray:
		std_msgs::ColorRGBA colorClusteredFrontier;
		colorClusteredFrontier.r = 1.0;
		colorClusteredFrontier.g = 0.9;
		colorClusteredFrontier.b = 0.1;
		colorClusteredFrontier.a = 0.7;

		for (unsigned i= 0; i < frontierNodesVis.markers.size(); ++i)
		{
			double size = m_octree->getNodeSize(i);

			frontierNodesVis.markers[i].header.frame_id = m_worldFrameId;
			frontierNodesVis.markers[i].header.stamp = ros::Time::now();
			frontierNodesVis.markers[i].ns = "red";
			frontierNodesVis.markers[i].id = i;
			frontierNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
			frontierNodesVis.markers[i].scale.x = size;
			frontierNodesVis.markers[i].scale.y = size;
			frontierNodesVis.markers[i].scale.z = size;
			frontierNodesVis.markers[i].color = colorClusteredFrontier;

			if (frontierNodesVis.markers[i].points.size() > 0)
				frontierNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
			else
				frontierNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
		}
		m_markerClusteredFrontierPub.publish(frontierNodesVis);
	}

	void FrontierServer::publishBestFrontier()
	{
		m_logfile << "publishBestFrontier" << endl;
		visualization_msgs::Marker frontier_goal;
		std_msgs::ColorRGBA colorgoalFrontier;
		colorgoalFrontier.r = 1.0;
		colorgoalFrontier.g = 0.5;
		colorgoalFrontier.b = 1.0;
		colorgoalFrontier.a = 0.8;
		geometry_msgs::Point cubeCenter;
		cubeCenter.x = m_bestFrontierPoint.x();
		cubeCenter.y = m_bestFrontierPoint.y();
		cubeCenter.z = m_bestFrontierPoint.z();

		frontier_goal.points.push_back(cubeCenter);
		double size = m_octree->getNodeSize(m_explorationDepth);

		frontier_goal.header.frame_id = m_worldFrameId;
		frontier_goal.header.stamp = ros::Time::now();
		frontier_goal.ns = "red";
		frontier_goal.type = visualization_msgs::Marker::CUBE_LIST;
		frontier_goal.scale.x = size;
		frontier_goal.scale.y = size;
		frontier_goal.scale.z = size;
		frontier_goal.color = colorgoalFrontier;

		if (frontier_goal.points.size() > 0)
			frontier_goal.action = visualization_msgs::Marker::ADD;
		else
			frontier_goal.action = visualization_msgs::Marker::DELETE;

		m_bestFrontierPub.publish(frontier_goal);
	}

	void FrontierServer::publishUAVGoal(point3d goal)
	{
		// // Make sure that point is in the bounding box
		if (goal.x() < m_explorationMinX || 
			goal.x() > m_explorationMaxX ||
			goal.y() < m_explorationMinY || 
			goal.y() > m_explorationMaxY ||
			goal.z() < m_explorationMinZ || 
			goal.z() > m_explorationMaxZ) 
		{
			ROS_ERROR("Want to publish a goal out of the bounding box.");
			setPointAsInvalid(goal);
			m_currentGoalReached = true;
			return; 
		}	
		geometry_msgs::PoseStamped m_goal;
		m_goal.header.frame_id = m_worldFrameId;
		m_goal.header.stamp = ros::Time::now();

		m_goal.pose.position.x = goal.x();
		m_goal.pose.position.y = goal.y();
		m_goal.pose.position.z = goal.z();
		m_goal.pose.orientation.x = 0;
		m_goal.pose.orientation.y = 0;
		m_goal.pose.orientation.z = 0;
		m_goal.pose.orientation.w = 1;

		m_uavGoalPub.publish(m_goal);
		ROS_WARN_STREAM(goal.x() << " " << goal.y() << " " << goal.z() << " -> Goal published!");
	}
}