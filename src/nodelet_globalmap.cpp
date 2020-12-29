#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <utils/include/all_utils.h>
#include <ros/ros.h>
#include <glmapping/local2global.h>
#include <global_map_cartesian.h>
#include <msg_local2global.h>
#include <rviz_vis.h>
#include <global2occupancygrid2d.h>
#include <global2esdf.h>
#include <global2esdf3d.h>

namespace glmapping_ns
{

class GlobalMapNodeletClass : public nodelet::Nodelet
{
public:
    GlobalMapNodeletClass()  {;}
    ~GlobalMapNodeletClass() {;}
private:

    ros::Subscriber sub_from_local;
    global_map_cartesian* global_map;
    rviz_vis *globalmap_publisher;
    Global2OccupancyGrid2D *occupancy_grid_publisher;
    Global2ESDF *esfd2d_publisher;
    Global2ESDF3DPatch *esfd3d_publisher;
    ros::Time last_esft_stamp;

    void from_lm_callback(const glmapping::local2globalConstPtr& msg)
    {
        SE3 T_wl;
        vector<Vec3> l2g_obs_l;
        vector<Vec3> l2g_miss_l;
        ros::Time stamp;
        msg_local2global::unpack(msg,T_wl,l2g_obs_l,l2g_miss_l,stamp);
        global_map->input_pc_pose(l2g_obs_l,l2g_miss_l,T_wl);
        globalmap_publisher->pub_globalmap(global_map->visualization_cell_list,stamp);
        occupancy_grid_publisher->pub_occupancy_grid_2D_from_globalmap(*global_map,stamp);
        esfd3d_publisher->pub_ESDF_3D_from_globalmap(*global_map,stamp);
        if((ros::Time::now().toSec()-last_esft_stamp.toSec())>0.19)
        {
            esfd2d_publisher->pub_ESDF_2D_from_globalmap(*global_map,stamp);
        }
    }

    virtual void onInit()
    {
        cout << "globalmapnode:" << endl;
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        string configFilePath;
        cout << "this is glmapping global node" << endl;
        nh.getParam("/glmapping_configfile",   configFilePath);
        cout << "read the config file" << endl;
        double d_x  = getDoubleVariableFromYaml(configFilePath,"glmapping_gm_d_x");
        double d_y  = getDoubleVariableFromYaml(configFilePath,"glmapping_gm_d_y");
        double d_z  = getDoubleVariableFromYaml(configFilePath,"glmapping_gm_d_z");
        int    n_x  = getIntVariableFromYaml(configFilePath,"glmapping_gm_n_x");
        int    n_y  = getIntVariableFromYaml(configFilePath,"glmapping_gm_n_y");
        int    n_z  = getIntVariableFromYaml(configFilePath,"glmapping_gm_n_z");
        double min_z       = getDoubleVariableFromYaml(configFilePath,"glmapping_gm_min_z");
        int    measure_cnt = getIntVariableFromYaml(configFilePath,"glmapping_gm_measurement_cnt");
        double occupied_sh = getDoubleVariableFromYaml(configFilePath,"glmapping_gm_occupied_p_sh");
        double free_sh     = getDoubleVariableFromYaml(configFilePath,"glmapping_gm_free_p_sh");


        global_map = new global_map_cartesian();
        global_map->init_map(d_x,d_y,d_z,n_x,n_y,n_z,min_z,
                             measure_cnt,occupied_sh,free_sh);
        double max_z=min_z+(d_z*n_z);
        globalmap_publisher =  new rviz_vis(nh,"/globalmap","map",2,min_z,max_z,d_x,d_z);
        occupancy_grid_publisher = new Global2OccupancyGrid2D(nh,"/occupancygrid",2);
        occupancy_grid_publisher->setGlobalMap(*global_map,"map");
        esfd2d_publisher = new Global2ESDF(nh,"/esfd_map",2);
        esfd2d_publisher->setGlobalMap(*global_map,"map");
        esfd3d_publisher = new Global2ESDF3DPatch(nh,"/esfd_batch",2);
        esfd3d_publisher->setGlobalMap(*global_map,"map");
        last_esft_stamp = ros::Time::now();
        sub_from_local = nh.subscribe<glmapping::local2global>(
                    "/local2global",
                    10,
                    boost::bind(&GlobalMapNodeletClass::from_lm_callback, this, _1));
        
    }

};//class GlobalMapNodeletClass
}//namespace glmapping_ns

PLUGINLIB_EXPORT_CLASS(glmapping_ns::GlobalMapNodeletClass, nodelet::Nodelet)


