#ifndef GLOBAL2ESFD_H
#define GLOBAL2ESFD_H

#include <nav_msgs/OccupancyGrid.h>
#include <global_map_cartesian.h>
#include <utils/include/all_utils.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>



#define ESDF_OCCUPIED (-1.0)
#define ESDF_UNKNOWN  (-2.0)
#define ESDF_SERCHRANGE (15)
typedef Matrix<double, Dynamic, Dynamic> MatrixXd;

class Global2ESDF
{
    int map2d_nx;
    int map2d_ny;
    int map2mat_bias_x;
    int map2mat_bias_y;
    double mat2vismap_bias_x;
    double mat2vismap_bias_y;
    double map2d_dx;
    double map2d_dy;


    vector<MatrixXd> esdf_map3d;
    MatrixXd esdf_map;


    visualization_msgs::Marker cubes_array;

    Vec3 esdf_cube_coler(double ratio);
public:
    ros::Publisher esdf_pub;
    
    Global2ESDF(ros::NodeHandle& nh, string topic_name, int buffersize=2);
    void setGlobalMap(global_map_cartesian &map, string world_fram_name="world");
    void pub_ESDF_2D_from_globalmap(global_map_cartesian &map, ros::Time stamp=ros::Time::now());

};

#endif // GLOBAL2ESFD_H
