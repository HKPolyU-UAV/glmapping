#ifndef LOCAL_MAP_CYLINDRICAL_H
#define LOCAL_MAP_CYLINDRICAL_H

#include <utils/include/all_utils.h>
#include <cylindrical_cell.h>
#include <glmapping/local2global.h>

class local_map_cylindrical
{
private:
    int nRho_x_nPhi;
    double z_border_min;
    double z_border_max;
    SE3 T_bs; //Transformation from sensor to body
    bool visibility_check;

public:

    //Properties
    double map_dRho;
    double map_dPhi;
    double map_dZ;
    int map_nRho;
    int map_nPhi;
    int map_nZ;
    int map_center_z_idx;
    double observation_range;

    //Map
    vector<cylindrical_cell> map;
    vector<cylindrical_cell> map_tmp;
    SE3 last_T_wl;
    bool first_input;

    //Init
    local_map_cylindrical();
    void setTbs(SE3 T_bs_in);
    void init_map(double d_Rho, double d_Phi_deg, double d_Z, int n_Rho, int n_z_below, int n_z_over);
    void creat_map();
    void creat_transfer_chart();

    //Visit certain cell
    size_t mapIdx(Vec3I Rho_Phi_z);
    size_t mapIdx(int Rho, int Phi, int z);
    Vec3I xyz2RhoPhiZ(Vec3 xyz_l);
    bool  xyz2RhoPhiZwithBoderCheck(Vec3 xyz_l, Vec3I &rhophiz);

    //local to global messages
    vector<Vec3> l2g_msg_obs_pts_l;//pts, which increase measurement times in the global map
    vector<Vec3> l2g_msg_miss_pts_l;//pts, which decrease measurement times in the global map
    SE3 l2g_msg_T_wl;//Tranformation for local to world frame

    //Visualization pts
    vector<Vec3> visualization_cell_list;

    //input callback
    void input_pc_pose(vector<Vec3> PC_s, SE3 T_wb);
};

#endif // LOCAL_MAP_CYLINDRICAL_H
