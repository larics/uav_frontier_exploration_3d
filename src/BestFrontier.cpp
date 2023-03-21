/*
 * BestFrontier.cpp
 *
 *  Created on: July 14, 2020
 *      Author: Ana Batinovic
 */

#include <uav_frontier_exploration_3d/BestFrontier.h>

namespace best_frontier
{
	BestFrontier::BestFrontier()
	{
		ros::NodeHandle private_nh {ros::NodeHandle("~")};
		
		m_logfile.open("/log_best_frontier.txt");
		m_logfile << "This is a log file for BestFrontier" << endl;

		// Read from yaml file
		private_nh.param("exploration_config_filename", m_configFilename, m_configFilename);
		configureFromFile(m_configFilename);
	}

	bool BestFrontier::configureFromFile(string config_filename)
	{
		cout << "BestFrontier - Configuring sensor specifications from file: " << endl;
		cout << "  " << config_filename << endl;
		
		// Open yaml file with configuration
		YAML::Node config = YAML::LoadFile(config_filename);

		// Get params
		m_resolution = config["octomap"]["resolution"].as<double>();
		m_boxInfGainSize = config["exploration"]["box_length"].as<double>();
		m_kGain = config["exploration"]["k_gain"].as<double>();
		m_lambda = config["exploration"]["lambda"].as<double>();
		
		return true;
	}

	point3d BestFrontier::bestFrontierInfGain(
		octomap::OcTree* octree,  point3d currentPosition, KeySet& Cells)
	{
		// m_logfile << "bestFrontierInfGain" << endl;
		ros::WallTime startTime_frontier = ros::WallTime::now();
		if (Cells.size() == 0) 
		{
			ROS_WARN("BestFrontier - Zero clustered frontiers!");
			return {};
		}
		vector<pair<point3d, point3d>> candidates;

		for(KeySet::iterator iter = Cells.begin(), 
			end = Cells.end(); iter != end; ++iter)
		{
			// Get cell position
			point3d tempCellPosition = octree->keyToCoord(*iter);
			double x = tempCellPosition.x();
			double y = tempCellPosition.y();
			double z = tempCellPosition.z();
			
			candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, 0.0)));	
		}

		// If the cluster point is not in octree
		if (candidates.size() == 0)
		{
			ROS_WARN("BestFrontier - Zero candidates.");	
			return {};
		} 
		// Calculate information gain for every clustered candidate
		std::vector<double> InfGainVector(candidates.size());
	
		for(int i = 0; i < candidates.size(); i++)
		{
			// Get candidate
			auto currCandidate = candidates[i];
			double unknownVolume = calcMIBox(octree, currCandidate.first);
			double tempDistance = calculateDistance(currentPosition, currCandidate.first);
			double kGain = m_kGain;
			InfGainVector[i] = kGain * unknownVolume * exp(- m_lambda * tempDistance); 
		}

		// Find max element index
		int maxElementIndex = 
			max_element(InfGainVector.begin(), InfGainVector.end()) - InfGainVector.begin();
	
		// Define best frontier
		point3d bestFrontier = point3d(
			candidates[maxElementIndex].first.x(),
			candidates[maxElementIndex].first.y(),
			candidates[maxElementIndex].first.z());
		m_logfile << "Best frontier point: " << bestFrontier << endl;
		double total_time_frontier = (ros::WallTime::now() - startTime_frontier).toSec();
		m_logfile << "Best frontier used total: "<< total_time_frontier << " sec" << endl;

		return bestFrontier;
	}

	double BestFrontier::calcMIBox(const octomap::OcTree *octree, const point3d &sensorOrigin)
	{
		ros::WallTime startTime_frontier = ros::WallTime::now();
		// Calculate number of unchanged cells inside a box around candidate point
		// Propotional to number of the unknown cells inside a box
		
		// Set bounding box
		point3d minPoint, maxPoint;
		double a = m_boxInfGainSize;
		minPoint.x() = sensorOrigin.x() - (a / 2);
		minPoint.y() = sensorOrigin.y() - (a / 2);
		minPoint.z() = sensorOrigin.z() - (a / 2);

		maxPoint.x() = sensorOrigin.x() + (a / 2);
		maxPoint.y() = sensorOrigin.y() + (a / 2);
		maxPoint.z() = sensorOrigin.z() + (a / 2);

		int unknownNum {0};
		int allNum {0};
		for(double ix = minPoint.x(); ix < maxPoint.x(); ix += m_resolution)
		{
			for(double iy = minPoint.y(); iy < maxPoint.y(); iy += m_resolution)
			{
				for (double iz = minPoint.z(); iz < maxPoint.z(); iz += m_resolution)
				{
					allNum++;
					if(!octree->search(ix, iy, iz))
						unknownNum++;
				}
			}
		}	
		return (double)unknownNum / (double)allNum;
	}

}   