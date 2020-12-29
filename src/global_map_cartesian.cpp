#include "global_map_cartesian.h"

double prob2logit(double prob)
{
    double odds = prob/(1-prob);
    double logit = log(odds);
    return logit;
}

double logit2prob(double logit)
{
    double odds = exp (logit);
    double prob = odds/(1+odds);
    return prob;
}

global_map_cartesian::global_map_cartesian()
{

}

size_t global_map_cartesian::mapIdx(Vec3I xyz_idx)
{
    return this->mapIdx(xyz_idx(0),xyz_idx(1),xyz_idx(2));
}
size_t global_map_cartesian::mapIdx(int x_idx, int y_idx, int z_idx)
{
    return static_cast<size_t>( z_idx*this->map_nx_times_ny
                                +y_idx*this->map_nx
                                +x_idx);
}

void global_map_cartesian::setTbs(SE3 T_bs_in)
{
    this->T_bs=T_bs_in;
}

void global_map_cartesian::init_map(double d_x, double d_y, double d_z,
                                    int n_x, int n_y, int n_z, double min_z,
                                    int measurement_volume,
                                    double occupied_sh, double free_sh)
{
    this->map_dx = d_x;
    this->map_dy = d_y;
    this->map_dz = d_z;
    this->map_nx = n_x;
    this->map_ny = n_y;
    this->map_nz = n_z;
    this->map_min_z = min_z;
    this->map_nx_times_ny = n_x*n_y;
    this->map_min_x = -(this->map_nx/2)*this->map_dx;
    this->map_min_y = -(this->map_ny/2)*this->map_dy;
    cout  << "map_nx"  <<  map_nx  <<endl;
    cout  << "map_ny"  <<  map_ny  <<endl;
    cout  << "map_nz"  <<  map_nz  <<endl;
    cout  << "min_z" <<  min_z << endl;
    this->creat_map();
    first_input = true;
    visibility_check = true;
    this->measurement_cnt_max = measurement_volume/2;
    this->measurement_cnt_min = -measurement_cnt_max;
    double occupied_logit_max = prob2logit(0.99);
    double free_logit_min     = prob2logit(0.01);
    cout << "occupied_logit_max: " << occupied_logit_max << endl;
    cout << "free_logit_min:     " << free_logit_min << endl;
    double ratio = measurement_cnt_max/occupied_logit_max;
    double occupied_logit = prob2logit(occupied_sh);
    double free_logit     = prob2logit(free_sh);
    double tuned_occupied_logit = occupied_logit*ratio;
    double tuned_free_logit     = free_logit*ratio;
    this->occupied_measurement_cnt = static_cast<int>(round(tuned_occupied_logit));
    this->free_measurement_cnt = static_cast<int>(round(tuned_free_logit));
    if(occupied_measurement_cnt > measurement_cnt_max) occupied_measurement_cnt = measurement_cnt_max;
    if(free_measurement_cnt < measurement_cnt_min) free_measurement_cnt = measurement_cnt_min;
    cout << "occupied_measurement_cnt:     " << occupied_measurement_cnt << endl;
    cout << "free_measurement_cnt: " << free_measurement_cnt << endl;
    cout << "----" << endl;
}

void global_map_cartesian::creat_map(void)
{

    for (int z=0; z<this->map_nz; z++)
    {
        for (int y=0; y<this->map_ny; y++)
        {
            for (int x=0; x<this->map_nx; x++)
            {
                double vis_x = this->map_min_x+(this->map_dx/2)+(x*this->map_dx);
                double vis_y = this->map_min_y+(this->map_dy/2)+(y*this->map_dy);
                double vis_z = this->map_min_z+(this->map_dz/2)+(z*this->map_dz);
                this->map.push_back(cartesian_cell(x,y,z,Vec3(vis_x,vis_y,vis_z)));
            }
        }
    }
    cout << map.size() << endl;
}

Vec3I global_map_cartesian::xyz2xyzIdx(Vec3 xyz_w)
{
    double x = xyz_w(0)-this->map_min_x;
    int x_idx =  static_cast<int>(x/this->map_dx);
    double y = xyz_w(1)-this->map_min_y;
    int y_idx =  static_cast<int>(y/this->map_dy);
    double z = xyz_w(2)-this->map_min_z;
    int z_idx =  static_cast<int>(z/this->map_dz);
    return Vec3I(x_idx,y_idx,z_idx);
}

bool global_map_cartesian::xyz2xyzIdxwithBoderCheck(Vec3 xyz_w, Vec3I &xyz_idx)
{
    double x = xyz_w(0)-this->map_min_x;
    int x_idx =  static_cast<int>(x/this->map_dx);
    double y = xyz_w(1)-this->map_min_y;
    int y_idx =  static_cast<int>(y/this->map_dy);
    double z = xyz_w(2)-this->map_min_z;
    int z_idx =  static_cast<int>(z/this->map_dz);
    if(x>0 && y>0 && z>0)
    {
        if( x>0 && y>0 && z>0
                && x_idx>=0 && y_idx>=0 && z_idx>=0
                && x_idx < this->map_nx && y_idx < this->map_ny && z_idx < this->map_nz)
        {
            xyz_idx = Vec3I(x_idx,y_idx,z_idx);
            return true;
        }
    }
    return false;
}

void global_map_cartesian::input_pc_pose(vector<Vec3> PC_l, vector<Vec3> PC_miss_l, SE3 T_wl)
{
    //Frame [w]orld, [s]ensor, [b]ody, [l]ocalmap;
    vector<Vec3> pc_w;
    vector<Vec3> pc_miss_w;
    for(auto p_l:PC_l)
    {
        pc_w.push_back(T_wl*p_l);
    }
    for(auto p_l:PC_miss_l)
    {
        pc_miss_w.push_back(T_wl*p_l);
    }
    //STEP 2: Add measurement
    for(auto p_w:pc_w)
    {
        Vec3I xyz_idx;
        if(xyz2xyzIdxwithBoderCheck(p_w,xyz_idx))
        {
            size_t map_idx=mapIdx(xyz_idx);
            map.at(map_idx).measurement_times++;
            if(map.at(map_idx).measurement_times > this->measurement_cnt_max)
            {
                map.at(map_idx).measurement_times=measurement_cnt_max;
            }
            //set observerable
            if(map.at(map_idx).measurement_times>this->occupied_measurement_cnt){
                map.at(this->mapIdx(xyz_idx)).is_occupied = true;
                map.at(this->mapIdx(xyz_idx)).sampled_xyz = p_w;
            }
        }else
        {
        }
    }
    for(auto p_miss_w:pc_miss_w)
    {
        Vec3I xyz_idx;
        if(xyz2xyzIdxwithBoderCheck(p_miss_w,xyz_idx))
        {
            size_t map_idx=mapIdx(xyz_idx);
            map.at(map_idx).measurement_times--;
            if(map.at(map_idx).measurement_times < this->measurement_cnt_min)
            {
                map.at(map_idx).measurement_times=measurement_cnt_min;
            }
            //set free
            if(map.at(map_idx).measurement_times < this->free_measurement_cnt){
                map.at(this->mapIdx(xyz_idx)).is_occupied = false;
            }
        }else
        {
        }
    }
    //Update occupied list;
    visualization_cell_list.clear();
    occupied_cell_idx_list.clear();
    for(auto cell:this->map)
    {
        if(cell.is_occupied)
        {
            occupied_cell_idx_list.push_back(Vec3I(cell.idx_x,cell.idx_y,cell.idx_z));
            visualization_cell_list.push_back(cell.vis_pt);
        }
    }
    this->newest_T_wl = T_wl;
    //cout << "globalmap vis size" << visualization_cell_list.size() << endl;

}
