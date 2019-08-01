// ros stuff
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

// service
#include "f110_occgrid/ConvertMap.h"
#include "f110_occgrid/gridmap_conversion.h"

// standard stuff
#include <math.h>
#include <vector>
#include <array>
#include <iostream>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// OpenCV
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

static const int STATIC_THRESH = 50;

class Gridmap {
public:
    Gridmap(ros::NodeHandle &nh);
    virtual ~Gridmap();
    Eigen::MatrixXi get_env_layer();
    Eigen::MatrixXi get_static_layer();
    Eigen::MatrixXi get_dynamic_layer();
    bool get_converted_image(f110_occgrid::ConvertMap::Request &req, f110_occgrid::ConvertMap::Response &res);
    sensor_msgs::Image get_img();
    cv::Mat get_cv_img(sensor_msgs::ImagePtr &image);
    cv::Mat get_cv_img();
    sensor_msgs::ImagePtr cv_2_ros_img(cv::Mat &img);
    void cv_2_ros_img(cv::Mat &img, ros::Publisher &img_pub);
    cv::Mat ros_2_cv_img(sensor_msgs::ImagePtr &img);
private:
    // ros stuff
    ros::NodeHandle nh_;
    ros::Publisher env_pub;
    ros::Publisher static_pub;
    ros::Publisher dynamic_pub;

    ros::Publisher env_viz_pub;
    ros::Publisher stat_viz_pub;
    ros::Publisher dyn_viz_pub;

    ros::ServiceServer service;

    ros::Subscriber scan_sub;
    ros::Subscriber map_sub;

    // tf stuff
    tf::TransformListener listener;

    // underlying data structures
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> env_layer;
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> static_layer;
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dynamic_layer;

    // map params
    float map_resolution; // m/cell
    int map_width, map_height; // cells
    geometry_msgs::Pose map_origin; // cell(0,0), [m, m, rad]
    float origin_x, origin_y;
    nav_msgs::MapMetaData all_map_metadata;
    nav_msgs::OccupancyGrid env_layer_msg;
    // int STATIC_THRESH; // threshold for value that's considered static obs

    // laser params
    bool LASER_INIT;
    std::vector<float> angles_vector;
    std::vector<float> current_scan;
    int SCAN_COUNT;

    // occgrid params
    int INFLATION;

    // current frame image
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;
    image_transport::Publisher layers_pub;
    sensor_msgs::ImagePtr current_img;
    int img_size;

    // gridmap converter
    GridmapConverter converter;

    // private methods
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void viz_layers();
    void pub_layers();
    void pub_layers(bool pub_image);
    void pub_layers(Eigen::MatrixXi &layer, ros::Publisher &publisher);
    cv::Rect get_roi(std::vector<int> &car_center);
    sensor_msgs::ImagePtr layers_2_img();
    cv::Mat layers_2_cv_img();
    cv::Mat transform_img(cv::Mat &full_img);
    void update_img(cv::Mat &img);
    void update_img(sensor_msgs::ImagePtr &img);
    std::vector<int> find_nonzero(Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> &arr);
    std::vector<int> ind_2_rc(int ind);
    int rc_2_ind(int r, int c);
    bool out_of_bounds(int x, int y);
    geometry_msgs::Point cell_2_coord(int row, int col);
    geometry_msgs::Point cell_2_coord(int ind);
    int coord_2_cell_ind(double x, double y);
    std::vector<int> coord_2_cell_rc(double x, double y);
};

