/*
 * ClusteringAlgorithm.h
 *
 *  Created on: May 11, 2020
 *      Author: Ana Batinovic
 */
#ifndef CLUSTERING_H
#define CLUSTERING_H

#include <ros/ros.h>
#include <vector>
#include <stdio.h>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <geometry_msgs/Point.h>

using namespace std;

//Mean shift clustering
#include <mean_shift_clustering/MeanShiftAlgorithm.h>

class MSCluster : public MeanShift {
  public:
    void getMeanShiftClusters(vector<geometry_msgs::Point> &points,
      vector<geometry_msgs::Point> &cluster_points, double kernelBandwidth)
    {
      ros::WallTime startTimeEvaluation = ros::WallTime::now();      
      vector<Cluster> clusters = cluster(points, kernelBandwidth);
      double totalTimeEvaluation = (ros::WallTime::now() - startTimeEvaluation).toSec();
      printf("Found %lu clusters\n", clusters.size());
      printf("====================\n");
      
      for (int i = 0 ; i < clusters.size(); i++)
      {
        // Find the nearest original point from cluster
        // Convert to point and send to calculateDistance
        geometry_msgs::Point tempPoint;
        tempPoint.x = clusters[i].mode.x;
        tempPoint.y = clusters[i].mode.y;
        tempPoint.z = clusters[i].mode.z;
        std::vector<double> vectorOfDistances {};
        for (int j = 0; j < points.size(); j++)
        {
          vectorOfDistances.push_back(calculateDistance(tempPoint, points[j]));
        }
        // Find min element
        int minElementIndex = std::min_element(
        vectorOfDistances.begin(), vectorOfDistances.end()) - vectorOfDistances.begin();
        cluster_points.push_back(points[minElementIndex]);
      }
    }

    double calculateDistance(const geometry_msgs::Point& p1,
      const geometry_msgs::Point& p2)
    {		
	    double distance = sqrt(
			pow(p2.x - p1.x, 2) +
			pow(p2.y - p1.y, 2) +
			pow(p2.z - p1.z, 2));

	    return distance;
    }  
};
#endif //CLUSTERING_H