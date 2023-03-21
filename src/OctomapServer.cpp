/*
 * OctomapServer.cpp
 *
 *  Created on: July 14, 2020
 *      Author: Ana Batinovic
 */

#include <uav_frontier_exploration_3d/OctomapServer.h>

namespace octomap_server
{
	OctomapServer::OctomapServer()
	{
		ros::NodeHandle private_nh {ros::NodeHandle("~")};

		// Read from yaml file
		private_nh.param("exploration_config_filename", m_configFilename, m_configFilename);
		configureFromFile(m_configFilename);
		
		m_logfile.open("log_octomap.txt");
		m_logfile << "This is a log file for OctomapServer." << endl;

		// Initialize octomap object and params
		m_octree = new OcTree(m_resolution);
		m_octree->setProbHit(m_probHit);
		m_octree->setProbMiss(m_probMiss);
		m_octree->setClampingThresMin(m_thresMin);
		m_octree->setClampingThresMax(m_thresMax);
		m_treeDepth = m_octree->getTreeDepth();
		m_maxTreeDepth = m_treeDepth;

		// Initialize publishers
		m_markerOccPub = m_nh.advertise<
			visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, false);
		m_markerFreePub = m_nh.advertise<
			visualization_msgs::MarkerArray>("free_cells_vis_array", 1, false);
		m_binaryMapPub = m_nh.advertise<Octomap>("octomap_binary", 1, false);
		m_pubVolumes = m_nh. advertise<
			std_msgs::Float64MultiArray>("octomap_volume", 1);

		// Initialize subscribers
		m_pointCloudSub = m_nh.subscribe("cloud_in", 1, 
			&OctomapServer::pointCloudCallback, this);
		m_uavGlobalPoseSub = m_nh.subscribe("odometry", 1, 
			&OctomapServer::globalPoseCallback, this);	
		m_saveOctomapServer = m_nh.advertiseService(
    	"exploration/save_octomap", &OctomapServer::saveOctomapServiceCb, this);	
	}

	OctomapServer::~OctomapServer()
	{
		if (m_octree)
		{
			delete m_octree;
			m_octree = NULL;
		}	
	}

	bool OctomapServer::configureFromFile(string config_filename)
	{
		cout << "OctomapServer - Configuring uav exploration from file: " << endl;
		cout << "  " << config_filename << endl;
		
		// Open yaml file with configuration
		YAML::Node config = YAML::LoadFile(config_filename);

		// Get params
		m_worldFrameId = config["exploration"]["global_frame"].as<string>();
		m_baseFrameId = config["exploration"]["base_link_frame"].as<string>();
		m_totalVol = config["exploration"]["volume"].as<double>();

		m_maxRange = config["sensor_model"]["max_range"].as<double>();
		m_probHit = config["sensor_model"]["probability_hit"].as<double>();
		m_probMiss = config["sensor_model"]["probability_miss"].as<double>();
		m_thresMin = config["sensor_model"]["clamping_thres_min"].as<double>();
		m_thresMax = config["sensor_model"]["clamping_thres_max"].as<double>();

		m_resolution = config["octomap"]["resolution"].as<double>();
		m_treeDepth = config["octomap"]["octree_depth"].as<unsigned>();
		m_filename = config["octomap"]["filename"].as<string>();
		m_filePath = config["octomap"]["file_path"].as<string>();

		return true;
	}

	void OctomapServer::pointCloudCallback(
		const sensor_msgs::PointCloud2::ConstPtr& cloud)
	{
		m_currPointCloud = *cloud;
		m_pointCloudReceivedFlag = true;
	}

	void OctomapServer::globalPoseCallback(
		const nav_msgs::Odometry::ConstPtr& msg)
	{
		m_uavCurrentPose = msg->pose.pose;
	}

	bool OctomapServer::saveOctomapServiceCb(
		std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
	{
		string filePath = "/home/" + string(getenv("USER")) + "/" + m_filePath;
		cout << "OctomapServer - saving octomap to file: " << filePath + m_filename << endl;

		if (m_octree == nullptr) 
			return false;

		m_octree->writeBinary(filePath + m_filename);
		return true;
	}

	// Function used to get all neighbours of a specific cell x, y, z
	void OctomapServer::genNeighborCoord(float x, float y, float z) 
	{
		point3d point(x, y, z);
		// The keys count the number of cells (voxels)
		// from the origin as discrete address of a voxel.
		OcTreeKey key;
		if (!m_octree->coordToKeyChecked(point, key)) 
		{
			OCTOMAP_ERROR_STR("Error in search: [" 
				<< point << "] is out of OcTree bounds!");
			return;
		}
		std::vector<octomap::point3d> neighbor;
		genNeighborCoord(key, neighbor);
	}

	void OctomapServer::genNeighborCoord(
		OcTreeKey start_key, std::vector<octomap::point3d>& occupiedNeighbor) 
	{
		occupiedNeighbor.clear();
		OcTreeKey neighbor_key;
		// 26 neighbours
		for (int i = 0; i < 26; i++) 
		{
			// Implements a lookup table that allows to
			// computer keys of neighbor cells directly
			m_lut.genNeighborKey(start_key, i, neighbor_key);
			point3d query = m_octree->keyToCoord(neighbor_key);
			occupiedNeighbor.push_back(query);
		}
	}

	void OctomapServer::trackChanges(PCLPointCloudI& changedCells) 
	{
		m_logfile << "OctomapServer - trackChanges" << endl;
		KeyBoolMap::const_iterator startPnt = m_octree->changedKeysBegin();
		KeyBoolMap::const_iterator endPnt = m_octree->changedKeysEnd();

		int c = 0;
		for (KeyBoolMap::const_iterator iter = startPnt; iter != endPnt; ++iter) 
		{
			c++;
			OcTreeNode* node = m_octree->search(iter->first);

			bool occupied = m_octree->isNodeOccupied(node);

			pcl::PointXYZI pnt;

			pnt.x = m_octree->keyToCoord(iter->first.k[0]);
			pnt.y = m_octree->keyToCoord(iter->first.k[1]);
			pnt.z = m_octree->keyToCoord(iter->first.k[2]);

			if (occupied) pnt.intensity = 1000;
			else pnt.intensity = -1000;

			changedCells.push_back(pnt);
		}
		m_octree->resetChangeDetection();
	}

	void OctomapServer::insertScan(const PCLPointCloud& cloud)
	{
		ros::WallTime startTime_insert = ros::WallTime::now();
		m_logfile << "OctomapServer - insertScan" << endl;
		
		// Set sensorOriginAsCurrentUAVPosition
		point3d sensorOrigin(m_uavCurrentPose.position.x, 
			m_uavCurrentPose.position.y, m_uavCurrentPose.position.z);

		if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)||
			!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
		{
			ROS_ERROR_STREAM("OctomapServer - Could not generate Key for origin " << sensorOrigin);
		}

		KeySet free_cells, occupied_cells;

		for (PCLPointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it)
		{
			point3d point(it->x, it->y, it->z);
			// Maxrange check
			if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) 
			{
				// Free cells
				if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
				{
					free_cells.insert(m_keyRay.begin(), m_keyRay.end());  
				}
				// Occupied endpoint
				OcTreeKey key;
				if (m_octree->coordToKeyChecked(point, key))
				{
					occupied_cells.insert(key);

					updateMinKey(key, m_updateBBXMin);
					updateMaxKey(key, m_updateBBXMax);
				}
			}
			else 
			{// Ray longer than maxrange:;
				point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
				if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay))
				{
					free_cells.insert(m_keyRay.begin(), m_keyRay.end());

					octomap::OcTreeKey endKey;
					if (m_octree->coordToKeyChecked(new_end, endKey))
					{
						updateMinKey(endKey, m_updateBBXMin);
						updateMaxKey(endKey, m_updateBBXMax);
					} 
					else
					{
						ROS_ERROR_STREAM("OctomapServer - Could not generate Key for endpoint "<<new_end);
					}
				}
			}
		}

		// Mark free cells only if not seen occupied in this cloud
		for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it)
		{
			if (occupied_cells.find(*it) == occupied_cells.end())
				m_octree->updateNode(*it, false);
		}

		// Mark all occupied cells:
		for (KeySet::iterator it = occupied_cells.begin(), end=free_cells.end(); it!= end; it++) 
		{
			m_octree->updateNode(*it, true);
		}

		octomap::point3d minPt, maxPt;

		minPt = m_octree->keyToCoord(m_updateBBXMin);
		maxPt = m_octree->keyToCoord(m_updateBBXMax);

		double total_time_insert = (ros::WallTime::now() - startTime_insert).toSec();
		m_logfile << "OctomapServer -InsertScan used total " << total_time_insert << " sec" << endl;
			
		if (m_compressMap)
			m_octree->prune();
		ros::WallTime startTime_evaluation = ros::WallTime::now();
		// Extract frontiers
		// Get changed cells in the current iteration
		m_octree->enableChangeDetection(true);
		m_changedCells.clear();
		trackChanges(m_changedCells);
		// cout << "OctomapServer - changed cells: " << m_changedCells.size() << endl;
	}

	void OctomapServer::runDefault()
	{
		ros::spinOnce(); 	
		tf::StampedTransform sensorToWorldTf;
		PCLPointCloud pc;
		
		transformAndFilterPointCloud(sensorToWorldTf, pc);
		insertScan(pc);
		publishOccAndFree();
		publishBinaryOctoMap();
	}
	
	void OctomapServer::transformAndFilterPointCloud(
		tf::StampedTransform& sensorToWorldTf, PCLPointCloud& pc)
	{
		while (!m_pointCloudReceivedFlag) 
		{
			ros::spinOnce();
			ros::Duration(0.2).sleep();
		}
		// Input cloud for filtering and ground-detection
		pcl::fromROSMsg(m_currPointCloud, pc);

		try 
		{
			m_tfListener.lookupTransform(
				m_worldFrameId, m_currPointCloud.header.frame_id,
				m_currPointCloud.header.stamp, sensorToWorldTf);
		} catch(tf::TransformException& ex){
			ROS_ERROR_STREAM( "Transform error of sensor data: "
				<< ex.what() << ", quitting callback");
			return;
		}

		Eigen::Matrix4f sensorToWorld;
		pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

		// Set up filter for height range, also removes NANs
		// TODO: set params!
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setFilterFieldName("z");
		pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

		// Directly transform to map frame
		pcl::transformPointCloud(pc, pc, sensorToWorld);

		// Filter height range
		pass.setInputCloud(pc.makeShared());
		pass.filter(pc);
	}

	bool OctomapServer::isSpeckleNode(const OcTreeKey&nKey) const 
	{
		OcTreeKey key;
		bool neighborFound = false;
		for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2])
		{
			for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1])
			{
				for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0])
				{
					if (key != nKey)
					{
						OcTreeNode* node = m_octree->search(key);
						if (node && m_octree->isNodeOccupied(node))
						{
							// We have a neighbor => break!
							neighborFound = true;
						}
					}
				}
			}
		}
		return neighborFound;
	}

	void OctomapServer::publishVolume()
	{
		double occVol, freeVol {0};
		double totalVol, leftVol;

		for (OcTree::leaf_iterator it = m_octree->begin_leafs(), 
			end = m_octree->end_leafs(); it != end; ++it)
			{
				double voxelSize = m_octree->getNodeSize(m_maxTreeDepth);
				if(m_octree->isNodeOccupied(*it))
					occVol += pow(voxelSize, 3);
				else
					freeVol += pow(voxelSize, 3);		
		}
		totalVol = m_totalVol;
		leftVol = totalVol - (occVol + freeVol);

		std_msgs::Float64MultiArray allVolumes;
		allVolumes.data.resize(5);
		allVolumes.data[0] = occVol;
		allVolumes.data[1] = freeVol;
		allVolumes.data[2] = totalVol;
		allVolumes.data[3] = leftVol;
		allVolumes.data[4] = ros::Time::now().toSec();
	
		m_pubVolumes.publish(allVolumes);
	}

	void OctomapServer::publishOccAndFree()
	{
		ros::WallTime startTime = ros::WallTime::now();
		size_t octomapSize = m_octree->size();
		if (octomapSize <= 1)
		{
			ROS_WARN("Nothing to publish, octree is empty");
			return;
		}

		// Init markers for free space:
		visualization_msgs::MarkerArray freeNodesVis;
		// Each array stores all cubes of a different size, one for each depth level:
		freeNodesVis.markers.resize(m_treeDepth + 1);

		// Init markers:
		visualization_msgs::MarkerArray occupiedNodesVis;
		// Each array stores all cubes of a different size, one for each depth level:
		occupiedNodesVis.markers.resize(m_treeDepth + 1);

		// Traverse all leafs in the tree:
		for (OcTree::iterator it = m_octree->begin(m_maxTreeDepth),
			end = m_octree->end(); it != end; ++it)
		{
			if (m_octree->isNodeOccupied(*it))
			{
				double z = it.getZ();
				if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
				{
					double size = it.getSize();
					double x = it.getX();
					double y = it.getY();

					// Ignore speckles in the map:
					if (m_filterSpeckles && (it.getDepth() == m_treeDepth + 1) &&
						isSpeckleNode(it.getKey()))
					{
						ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
						continue;
					} // else: current octree node is no speckle, send it out
							
					// Create marker:
					unsigned idx = it.getDepth();
					assert(idx < occupiedNodesVis.markers.size());

					geometry_msgs::Point cubeCenter;
					cubeCenter.x = x;
					cubeCenter.y = y;
					cubeCenter.z = z;

					occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
				}
			} 
			else
			{ 
				// Node not occupied => mark as free in 2D map if unknown so far
				double z = it.getZ();
				if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
				{
				
					double x = it.getX();
					double y = it.getY();

					// Create marker for free space:
					unsigned idx = it.getDepth();
					assert(idx < freeNodesVis.markers.size());

					geometry_msgs::Point cubeCenter;
					cubeCenter.x = x;
					cubeCenter.y = y;
					cubeCenter.z = z;

					freeNodesVis.markers[idx].points.push_back(cubeCenter);
				}
			}
		}

		// Finish MarkerArray:
		std_msgs::ColorRGBA colorOcc, colorFree;
		colorOcc.r = 0.0;
		colorOcc.g = 0.0;
		colorOcc.b = 1.0;
		colorOcc.a = 1.0;

		colorFree.r = 0.0;
		colorFree.g = 1.0;
		colorFree.b = 0.0;
		colorFree.a = 1.0;
		for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i)
		{
			double size = m_octree->getNodeSize(i);

			occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
			occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
			occupiedNodesVis.markers[i].ns = "red";
			occupiedNodesVis.markers[i].id = i;
			occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
			occupiedNodesVis.markers[i].scale.x = size;
			occupiedNodesVis.markers[i].scale.y = size;
			occupiedNodesVis.markers[i].scale.z = size;
			occupiedNodesVis.markers[i].color = colorOcc;

			if (occupiedNodesVis.markers[i].points.size() > 0)
				occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
			else
				occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
		}
		m_markerOccPub.publish(occupiedNodesVis);

		// Finish FreeMarkerArray:
		for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i)
		{
			double size = m_octree->getNodeSize(i);

			freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
			freeNodesVis.markers[i].header.stamp = ros::Time::now();
			freeNodesVis.markers[i].ns = "red";
			freeNodesVis.markers[i].id = i;
			freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
			freeNodesVis.markers[i].scale.x = size;
			freeNodesVis.markers[i].scale.y = size;
			freeNodesVis.markers[i].scale.z = size;
			freeNodesVis.markers[i].color = colorFree;

			if (freeNodesVis.markers[i].points.size() > 0)
				freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
			else
				freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
		}
		m_markerFreePub.publish(freeNodesVis);
	}

	void OctomapServer::publishBinaryOctoMap()
	{
		Octomap map;
		map.header.frame_id = m_worldFrameId;
		map.header.stamp = ros::Time::now();

		if (octomap_msgs::binaryMapToMsg(*m_octree, map))
			m_binaryMapPub.publish(map);
		else
			ROS_ERROR("Error serializing OctoMap");
	}
}