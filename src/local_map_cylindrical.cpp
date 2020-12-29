#include "local_map_cylindrical.h"

local_map_cylindrical::local_map_cylindrical()
{

}

inline size_t local_map_cylindrical::mapIdx(Vec3I Rho_Phi_z)
{
    return this->mapIdx(Rho_Phi_z(0),Rho_Phi_z(1),Rho_Phi_z(2));
}
inline size_t local_map_cylindrical::mapIdx(int Rho, int Phi, int z)
{
    return static_cast<size_t>(z*this->nRho_x_nPhi
                               +Phi*this->map_nRho
                               +Rho);
}

void local_map_cylindrical::setTbs(SE3 T_bs_in)
{
    this->T_bs=T_bs_in;
}

void local_map_cylindrical::init_map(double d_Rho, double d_Phi_deg, double d_Z, int n_Rho, int n_z_below, int n_z_over)
{
    this->map_dRho = d_Rho;
    this->map_dPhi = d_Phi_deg * M_PI / 180;
    this->map_dZ   = d_Z;

    this->map_nRho = n_Rho;
    this->map_nPhi = static_cast<int>(360/d_Phi_deg);
    this->map_nZ   = n_z_below+n_z_over;
    this->map_center_z_idx = n_z_below;
    this->z_border_min = -(n_z_below*d_Z);
    this->nRho_x_nPhi = map_nRho*map_nPhi;
    this->creat_map();
    map_tmp = map;
    first_input = true;
    visibility_check = true;
    this->observation_range = sqrt(pow(map_nRho*map_dRho,2)+pow(map_nZ*map_dZ,2));
}

void local_map_cylindrical::creat_map(void)
{
    for (int z=0; z<this->map_nZ; z++)
    {
        for (int phi=0; phi<this->map_nPhi; phi++)
        {
            for (int rho=0; rho<this->map_nRho; rho++)
            {
                double vis_z = this->z_border_min+(this->map_dZ/2)+(z*this->map_dZ);
                double vis_rho = this->map_dRho/2+(rho*this->map_dRho);
                double vis_phi = this->map_dPhi/2+(phi*this->map_dPhi);
                double vis_x = vis_rho*cos(vis_phi);
                double vis_y = vis_rho*sin(vis_phi);
                this->map.push_back(cylindrical_cell(rho,phi,z,Vec3(vis_x,vis_y,vis_z)));
            }
        }
    }
    cout << map.size() << endl;
}

Vec3I local_map_cylindrical::xyz2RhoPhiZ(Vec3 xyz_l)
{
    double rho = sqrt(pow(xyz_l(0),2)+pow(xyz_l(1),2));
    int rho_idx =  static_cast<int>(rho/this->map_dRho);
    double phi = atan2(xyz_l(1),xyz_l(0));
    if (phi<0) phi += 2*M_PI;
    int phi_idx = static_cast<int>(phi/this->map_dPhi);
    double z = xyz_l(2)-this->z_border_min;
    int z_idx = static_cast<int>(z/this->map_dZ);
    return Vec3I(rho_idx,phi_idx,z_idx);
}

bool local_map_cylindrical::xyz2RhoPhiZwithBoderCheck(Vec3 xyz_l, Vec3I &rhophiz)
{
    double rho = sqrt(pow(xyz_l(0),2)+pow(xyz_l(1),2));
    int rho_idx =  static_cast<int>(rho/this->map_dRho);
    double phi = atan2(xyz_l(1),xyz_l(0));
    if (phi<0) phi += 2*M_PI;
    int phi_idx = static_cast<int>(phi/this->map_dPhi);
    double z = xyz_l(2)-this->z_border_min;
    int z_idx = static_cast<int>(z/this->map_dZ);
    rhophiz = Vec3I(rho_idx,phi_idx,z_idx);
    if(     rho_idx>0
            && phi_idx>0
            && z_idx>0
            && rho_idx < this->map_nRho
            && phi_idx < this->map_nPhi
            && z_idx < this->map_nZ)
    {
        return true;
    }
    return false;
}

void local_map_cylindrical::input_pc_pose(vector<Vec3> PC_s, SE3 T_wb)
{
    //Frame [w]orld, [s]ensor, [b]ody, [l]ocalmap;
    //cout << PC_s.size() << endl;
    SE3 T_wl(SO3(Quaterniond(1,0,0,0)),T_wb.translation());
    SE3 T_ws = T_wb * this->T_bs;
    SE3 T_ls = T_wl.inverse() * T_ws;
    vector<Vec3> PC_l;
    for(auto p_s:PC_s)
    {
        PC_l.push_back(T_ls*p_s);
    }
    //STEP 1: Transfer the local map.
    //    for(auto& grid:this->map)
    //    {
    //        grid.is_occupied = false;
    //    }

    if(first_input)
    {
        first_input = false;
        //do nothing
    }else
    {
        map_tmp = map;
        for(auto& cell:this->map)
        {
            cell.is_occupied = false;
        }
        Vec3 t_diff = -(T_wl.translation()-this->last_T_wl.translation());
        for(auto& cell:this->map_tmp)
        {
            if(cell.is_occupied)
            {
                Vec3 transfered_pt_l = cell.sampled_xyz + t_diff;
                Vec3I rpz_idx;
                if(xyz2RhoPhiZwithBoderCheck(transfered_pt_l,rpz_idx))
                {//set observerable
                    map.at(this->mapIdx(rpz_idx)).is_occupied = true;
                    map.at(this->mapIdx(rpz_idx)).sampled_xyz = transfered_pt_l;
                }
            }
        }
    }

    //STEP 2: Add measurement
    l2g_msg_obs_pts_l.clear();
    l2g_msg_miss_pts_l.clear();
    l2g_msg_T_wl = T_wl;
    for(auto p_l:PC_l)
    {
        Vec3I rpz_idx;
        if(p_l.norm()<observation_range)
        {
            l2g_msg_obs_pts_l.push_back(p_l);
        }
        if(xyz2RhoPhiZwithBoderCheck(p_l,rpz_idx))
        {
            //set observerable
            map.at(this->mapIdx(rpz_idx)).is_occupied = true;
            map.at(this->mapIdx(rpz_idx)).sampled_xyz = p_l;
            if(visibility_check)
            {
                float landing_rate = map.at(this->mapIdx(rpz_idx)).landing_rate_to_sensor;
                for (int r=rpz_idx[0]-1; r>0 ; r--) {
                    int rpz_z = static_cast<int>(r*landing_rate);
                    map.at(this->mapIdx(Vec3I(r,rpz_idx[1],rpz_z))).is_occupied = false;
                    l2g_msg_miss_pts_l.push_back(map.at(this->mapIdx(Vec3I(r,rpz_idx[1],rpz_z))).vis_pt);
                }
            }
        }else
        {
        }
    }
    //Update visualization list;
    visualization_cell_list.clear();
    for(auto cell:this->map)
    {
        if(cell.is_occupied)
        {
            visualization_cell_list.push_back(cell.vis_pt);
        }
    }
    this->last_T_wl = T_wl;
    //cout << "local map vis size" << visualization_cell_list.size() << endl;

}
