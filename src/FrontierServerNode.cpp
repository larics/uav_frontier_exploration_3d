#include <ros/ros.h>
#include <uav_frontier_exploration_3d/FrontierServer.h>

using namespace frontier_server;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_server");
    std::shared_ptr<FrontierServer> esmObj{
        new FrontierServer};
    esmObj->run();
    return 0;
}