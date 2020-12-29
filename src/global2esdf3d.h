#ifndef GLOBAL2ESDF3D_H
#define GLOBAL2ESDF3D_H

#include <nav_msgs/OccupancyGrid.h>
#include <global_map_cartesian.h>
#include <utils/include/all_utils.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>


#define ESDF_BATCH_SIZE_NX_NY (35)
#define ESDF_BATCH_SIZE_NZ (11)
#define ESDF_BATCH_MAX_SERCH_RANGE (10)

#define ESDF_BATCH_OCCUPIED (0.0)
typedef Matrix<double, Dynamic, Dynamic> MatrixXd;
typedef Matrix<bool,   Dynamic, Dynamic> MatrixXb;

class Global2ESDF3DPatch
{
    int map3d_nx;
    int map3d_ny;
    int map3d_nz;
    int map2mat_bias_x;
    int map2mat_bias_y;
    double mat2vismap_bias_x;
    double mat2vismap_bias_y;
    double map3d_dx;
    double map3d_dy;
    double map3d_dz;
    double esdf_cutoff_value;

    int visualized_layer;
    vector<MatrixXd> esdf_map3d;

    visualization_msgs::Marker cubes_array;

    Vec3 esdf_cube_coler(double ratio);
public:
    ros::Publisher esdf_pub;
    
    Global2ESDF3DPatch(ros::NodeHandle& nh, string topic_name, int buffersize=2);
    void setGlobalMap(global_map_cartesian &map, string world_fram_name="world");
    void pub_ESDF_3D_from_globalmap(global_map_cartesian &map, ros::Time stamp=ros::Time::now());

};

#endif // GLOBAL2ESFD_H
