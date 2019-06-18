#include "f110_occgrid/gridmap_viz.h"

GridmapViz::~GridmapViz() {
    ROS_INFO("Occupancy grid visualization shutting down");
}

GridmapViz::GridmapViz(ros::NodeHandle &nh) : nh_(nh) {
    env_viz_pub = nh_.advertise<visualization_msgs::Marker>("env_viz", 10);
    static_viz_pub = nh_.advertise<visualization_msgs::Marker>("static_viz", 10);
    dynamic_viz_pub = nh_.advertise<visualization_msgs::Marker>("dynamic_viz", 10);

    env_sub = nh_.subscribe("env_layer", 10, &GridmapViz::env_callback, this);
    static_sub = nh_.subscribe("static_layer", 10, &GridmapViz::static_callback, this);
    dynamic_sub = nh_.subscribe("dynamic_layer", 10, &GridmapViz::dynamic_callback, this);

    // called once
    boost::shared_ptr<nav_msgs::MapMetaData const> env_metadata_ptr;
    nav_msgs::MapMetaData env_metadata_msg;
    env_metadata_ptr = ros::topic::waitForMessage<nav_msgs::MapMetaData>("/map_metadata");
    if (env_metadata_ptr != NULL) {
        env_metadata_msg = *env_metadata_ptr;
    }
    map_resolution = env_metadata_msg.resolution;
    map_width = env_metadata_msg.width;
    map_height = env_metadata_msg.height;
    map_origin = env_metadata_msg.origin;
    geometry_msgs::Point origin = map_origin.position;
    origin_x = origin.x;
    origin_y = origin.y;
    // don't think orientation is used here
    ROS_INFO("Map Metadata Loaded");

    // STATIC_THRESH = 50;

    // making sure tf between map and laser is published before running
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/map", "/laser", now, ros::Duration(15.0));
    ROS_INFO("Transform arrived.");
}

// callbacks
void GridmapViz::env_callback(const nav_msgs::OccupancyGrid::ConstPtr& env_layer_msg) {
    std::vector<int8_t> env_layer_raw = env_layer_msg->data;
    std::vector<int> env_layer(env_layer_raw.begin(), env_layer_raw.end());
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.type = marker.CUBE_LIST;
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;

    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 0.0;
    col.g = 0.0;
    col.b = 0.0;

    for (int i=0; i<env_layer.size(); i++){
        if (env_layer[i] != 0) {
            geometry_msgs::Point cube = cell_2_coord(i);
            marker.points.push_back(cube);
            marker.colors.push_back(col);
        }
    }
    env_viz_pub.publish(marker);
}

void GridmapViz::static_callback(const nav_msgs::OccupancyGrid::ConstPtr& static_layer_msg){
    std::vector<int8_t> static_layer_raw = static_layer_msg->data;
    std::vector<int> static_layer(static_layer_raw.begin(), static_layer_raw.end());
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.type = marker.CUBE_LIST;
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;

    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 1.0;
    col.g = 0.0;
    col.b = 0.0;

    for (int i=0; i<static_layer.size(); i++){
        if (static_layer[i] >= STATIC_THRESH) {
            geometry_msgs::Point cube = cell_2_coord(i);
            marker.points.push_back(cube);
            marker.colors.push_back(col);
        }
    }
    static_viz_pub.publish(marker);
}

void GridmapViz::dynamic_callback(const nav_msgs::OccupancyGrid::ConstPtr& dynamic_layer_msg){
    std::vector<int8_t> dynamic_layer_raw = dynamic_layer_msg->data;
    std::vector<int> dynamic_layer(dynamic_layer_raw.begin(), dynamic_layer_raw.end());
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.type = marker.CUBE_LIST;
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;

    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 0.0;
    col.g = 0.0;
    col.b = 1.0;

    for (int i=0; i<dynamic_layer.size(); i++){
        if (dynamic_layer[i] != 0) {
            geometry_msgs::Point cube = cell_2_coord(i);
            marker.points.push_back(cube);
            marker.colors.push_back(col);
        }
    }
    dynamic_viz_pub.publish(marker);
}

std::vector<int> GridmapViz::ind_2_rc(int ind) {
    //[row, col]
    std::vector<int> rc;
    int row = floor(ind/map_width);
    int col = ind%map_width-1;
    rc.push_back(row);
    rc.push_back(col);
    return rc;
}

geometry_msgs::Point GridmapViz::cell_2_coord(int ind) {
    std::vector<int> rc = ind_2_rc(ind);
    geometry_msgs::Point coord;
    coord.x = origin_x + rc[1]*map_resolution;
    coord.y = origin_y + rc[0]*map_resolution;
    return coord;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "gridmap_viz");
    ros::NodeHandle nh;
    GridmapViz gridmap_vis(nh);
    ros::spin();
    return 0;
}
