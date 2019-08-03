#include "ros/ros.h"
#include "f110_occgrid/ConvertMap.h"
#include "f110_occgrid/gridmap.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_conversion_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<f110_occgrid::ConvertMap>("convert_map");
    f110_occgrid::ConvertMap srv;
    if (client.call(srv)) {
        ROS_INFO("returned true");
    } else {
        ROS_ERROR("returned false");
        return 1;
    }
    return 0;
}