#ifndef CARTESIAN_CELL_H
#define CARTESIAN_CELL_H
#include <utils/include/all_utils.h>

class cartesian_cell
{
public:
    int idx_x;
    int idx_y;
    int idx_z;
    Vec3 sampled_xyz;
    bool is_occupied;
    int  measurement_times;


    Vec3 vis_pt;
    double vis_radius;


    cartesian_cell();
    cartesian_cell(int x, int y, int z, Vec3 vis_xyz);

};

#endif // CYLINDERICAL_CELL_H
