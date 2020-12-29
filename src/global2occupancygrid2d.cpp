#include "global2occupancygrid2d.h"

Global2OccupancyGrid2D::Global2OccupancyGrid2D(ros::NodeHandle& nh, string topic_name, int buffersize)
{
    this->occupancygrid_pub = nh.advertise<nav_msgs::OccupancyGrid>(topic_name, buffersize);
}

void Global2OccupancyGrid2D::setGlobalMap(global_map_cartesian &map, string world_fram_name)
{

    this->map2d_nx = map.map_nx;
    this->map2d_ny = map.map_ny;
    this->map2d_dx = map.map_dx;
    this->map2d_dy = map.map_dy;

    occupancy_grid.header.frame_id = world_fram_name;

    occupancy_grid.info.origin.position.x = -(map2d_dx*map2d_nx)/2;
    occupancy_grid.info.origin.position.y = -(map2d_dy*map2d_ny)/2;
    occupancy_grid.data.clear();

    occupancy_grid.info.resolution = map2d_dx;         // float32
    occupancy_grid.info.width      = map2d_nx;           // uint32
    occupancy_grid.info.height     = map2d_ny;           // uint32

    for (int i = 0; i < map2d_nx*map2d_ny; i++)
    {
        occupancy_grid.data.push_back(0);
    }
}

void Global2OccupancyGrid2D::pub_occupancy_grid_2D_from_globalmap(global_map_cartesian &map, ros::Time stamp)
{

    occupancy_grid.header.stamp = stamp;
    //occupancy_grid.header.stamp = ros::Time::now();

    for(auto cell:map.occupied_cell_idx_list)
    {
        occupancy_grid.data.at(cell(0)+cell(1)*map2d_nx)=100;
    }

    this->occupancygrid_pub.publish(occupancy_grid);
}
