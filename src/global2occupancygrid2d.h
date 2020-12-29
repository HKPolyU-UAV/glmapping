#ifndef GLOBAL2OCCUPANCYGRID2D_H
#define GLOBAL2OCCUPANCYGRID2D_H

#include <nav_msgs/OccupancyGrid.h>
#include <global_map_cartesian.h>
#include <utils/include/all_utils.h>
#include <ros/ros.h>
#include <ros/publisher.h>

class Global2OccupancyGrid2D
{
    int map2d_nx;
    int map2d_ny;
    double map2d_dx;
    double map2d_dy;

    nav_msgs::OccupancyGrid occupancy_grid;


public:
    ros::Publisher occupancygrid_pub;
    
    Global2OccupancyGrid2D(ros::NodeHandle& nh, string topic_name, int buffersize=2);
    void setGlobalMap(global_map_cartesian &map, string world_fram_name="world");
    void pub_occupancy_grid_2D_from_globalmap(global_map_cartesian &map, ros::Time stamp=ros::Time::now());

};

#endif // GLOBAL2OCCUPANCYGRID2D_H
