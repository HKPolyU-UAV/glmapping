#ifndef CYLINDERICAL_CELL_H
#define CYLINDERICAL_CELL_H

#include <utils/include/all_utils.h>

class cylindrical_cell
{
public:
    int idx_rho;
    int idx_phi;
    int idx_z;
    Vec3 sampled_xyz;
    bool is_occupied;
    float  landing_rate_to_sensor; //landing rate refer to (Zpt-Zsensor)/Rho

    Vec3 vis_pt;
    //double vis_radius;

    cylindrical_cell();
    cylindrical_cell(int Rho, int Phi, int Z, Vec3 vis_xyz);

//    bool obervation_update(SE3 pose, Vec3 xyz_l);
//    bool occlusion_update();
};

#endif // CYLINDERICAL_CELL_H
