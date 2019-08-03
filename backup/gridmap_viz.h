#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include "f110_occgrid/gridmap.h"


#include <vector>

class GridmapViz {
public:
    GridmapViz(ros::NodeHandle &nh);
    virtual ~GridmapViz();
private:
    ros::NodeHandle nh_;
    ros::Publisher env_viz_pub;
    ros::Publisher static_viz_pub;
    ros::Publisher dynamic_viz_pub;

    ros::Subscriber env_sub;
    ros::Subscriber static_sub;
    ros::Subscriber dynamic_sub;

    float map_resolution;
    int map_width, map_height;
    float origin_x, origin_y;
    geometry_msgs::Pose map_origin;

    // tf stuff
    tf::TransformListener listener;

    // int STATIC_THRESH;

    void env_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void static_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void dynamic_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    std::vector<int> ind_2_rc(int ind);
    geometry_msgs::Point cell_2_coord(int ind);
};