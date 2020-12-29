#include "cartesian_cell.h"

cartesian_cell::cartesian_cell()
{

}

cartesian_cell::cartesian_cell(int x, int y, int z, Vec3 vis_xyz)
{
    this->idx_x = x;
    this->idx_y = y;
    this->idx_z = z;
    this->vis_pt=vis_xyz;
    this->is_occupied = false;
    this->measurement_times = 0;
}
