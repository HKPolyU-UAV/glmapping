#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <utils/include/all_utils.h>
#include <local_map_cylindrical.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rviz_vis.h>
#include <msg_local2global.h>

namespace glmapping_ns
{

using namespace std;

class LocalMapNodeletClass : public nodelet::Nodelet
{
public:
    LocalMapNodeletClass()  {;}
    ~LocalMapNodeletClass() {;}
private:

    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> ExactSyncPolicy;
    message_filters::Synchronizer<ExactSyncPolicy> * exactSync_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> ApproxSyncPolicy;
    message_filters::Synchronizer<ApproxSyncPolicy> * approxSync_;

    msg_local2global*      l2g_pub;
    local_map_cylindrical* local_map;
    bool enable_downsample;
    int  downsample_size;
    bool   publish_T_wb;
    bool   publish_T_bs;
    string frame_id;
    string body_frame_id;
    string sensor_frame_id;
    string local_frame_id;
    SE3 T_bs;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped_T_wb;
    geometry_msgs::TransformStamped transformStamped_T_wl;
    geometry_msgs::TransformStamped transformStamped_T_bs;
    rviz_vis *localmap_publisher;

    virtual void onInit()
    {
        cout << "localmapnode:" << endl;
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        string configFilePath;
        nh.getParam("/glmapping_configfile",   configFilePath);
        cout << "read the config file" << endl;

        double d_Rho          = getDoubleVariableFromYaml(configFilePath,"glmapping_lm_d_Rho");
        double d_Phi_deg      = getDoubleVariableFromYaml(configFilePath,"glmapping_lm_d_Phi_deg");
        double d_Z            = getDoubleVariableFromYaml(configFilePath,"glmapping_lm_d_Z");
        int    n_Rho          = getIntVariableFromYaml(configFilePath,"glmapping_lm_n_Rho");
        int    n_Z_below      = getIntVariableFromYaml(configFilePath,"glmapping_lm_n_Z_below");
        int    n_Z_over       = getIntVariableFromYaml(configFilePath,"glmapping_lm_n_Z_over");
        Mat4x4  T_bs_mat      = Mat44FromYaml(configFilePath,"T_B_S");
        bool use_exactsync    = getBoolVariableFromYaml(configFilePath,"use_exactsync");
        publish_T_wb          = getBoolVariableFromYaml(configFilePath,"publish_T_wb");
        publish_T_bs          = getBoolVariableFromYaml(configFilePath,"publish_T_bs");
        frame_id              = getStringFromYaml(configFilePath,"world_frame_id");
        body_frame_id         = getStringFromYaml(configFilePath,"body_frame_id");
        sensor_frame_id       = getStringFromYaml(configFilePath,"sensor_frame_id");
        local_frame_id        = getStringFromYaml(configFilePath,"localmap_frame_id");
        T_bs = SE3(T_bs_mat.topLeftCorner(3,3),
                   T_bs_mat.topRightCorner(3,1));
        localmap_publisher =  new rviz_vis(nh,"/localmap",local_frame_id,3,-n_Z_below*d_Z,n_Z_over*d_Z);
        l2g_pub =  new msg_local2global(nh,"/local2global",2);

        transformStamped_T_wb.header.frame_id = frame_id;
        transformStamped_T_wb.child_frame_id  = body_frame_id;
        transformStamped_T_wb.transform.translation.x = 0;
        transformStamped_T_wb.transform.translation.y = 0;
        transformStamped_T_wb.transform.translation.z = 0;
        transformStamped_T_wb.transform.rotation.x = 0;
        transformStamped_T_wb.transform.rotation.y = 0;
        transformStamped_T_wb.transform.rotation.z = 0;
        transformStamped_T_wb.transform.rotation.w = 1;
        transformStamped_T_wl=transformStamped_T_wb;
        transformStamped_T_wl.child_frame_id = local_frame_id;

        transformStamped_T_bs.header.frame_id = body_frame_id;
        transformStamped_T_bs.child_frame_id  = sensor_frame_id;
        transformStamped_T_bs.transform.translation.x = T_bs.translation().x();
        transformStamped_T_bs.transform.translation.y = T_bs.translation().y();
        transformStamped_T_bs.transform.translation.z = T_bs.translation().z();
        transformStamped_T_bs.transform.rotation.x = T_bs.so3().unit_quaternion().x();
        transformStamped_T_bs.transform.rotation.y = T_bs.so3().unit_quaternion().y();
        transformStamped_T_bs.transform.rotation.z = T_bs.so3().unit_quaternion().z();
        transformStamped_T_bs.transform.rotation.w = T_bs.so3().unit_quaternion().w();

        enable_downsample = true;
        downsample_size   = 1000;

        //init map
        local_map = new local_map_cylindrical();
        local_map->setTbs(T_bs);
        local_map->init_map(d_Rho,d_Phi_deg,d_Z,n_Rho,n_Z_below,n_Z_over);

        //init the local map

        if(use_exactsync)
        {
            pc_sub.subscribe(nh,   "/glmapping/pc", 10);
            pose_sub.subscribe(nh, "/glmapping/pose", 10);
            exactSync_ = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(5), pc_sub, pose_sub);
            exactSync_->registerCallback(boost::bind(&LocalMapNodeletClass::pc_pose_input_callback, this, _1, _2));
            cout << "ExactSyncPolicy" << endl;
        }
        else
        {
            pc_sub.subscribe(nh,   "/glmapping/pc", 1);
            pose_sub.subscribe(nh, "/glmapping/pose", 1);
            approxSync_ = new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(100), pc_sub, pose_sub);
            approxSync_->registerCallback(boost::bind(&LocalMapNodeletClass::pc_pose_input_callback, this, _1, _2));
            cout << "ApproxSyncPolicy" << endl;
        }
        ros::Rate rate(10.0);
        while(1)
        {
            transformStamped_T_wl.header.stamp = ros::Time::now();
            br.sendTransform(transformStamped_T_wl);
            if(publish_T_wb)
            {
                transformStamped_T_wb.header.stamp = ros::Time::now();
                br.sendTransform(transformStamped_T_wb);
            }
            if(publish_T_bs)
            {
                transformStamped_T_bs.header.stamp = ros::Time::now();
                br.sendTransform(transformStamped_T_bs);
            }
            rate.sleep();
        }
    }

    void pc_pose_input_callback(const sensor_msgs::PointCloud2::ConstPtr & pc_Ptr,
                                const geometry_msgs::PoseStamped::ConstPtr & pose_Ptr)
    {
        tic_toc_ros update_time;
//        static int i=0;
//        cout << "in the localmap callback " << i++ << endl;

        SE3 T_wb(SO3(Quaterniond(pose_Ptr->pose.orientation.w,
                                 pose_Ptr->pose.orientation.x,
                                 pose_Ptr->pose.orientation.y,
                                 pose_Ptr->pose.orientation.z)),
                 Vec3(pose_Ptr->pose.position.x,
                      pose_Ptr->pose.position.y,
                      pose_Ptr->pose.position.z));
        if(publish_T_wb)
        {
            transformStamped_T_wb.header.stamp = pose_Ptr->header.stamp;
            transformStamped_T_wb.header.frame_id = frame_id;
            transformStamped_T_wb.child_frame_id  = body_frame_id;
            transformStamped_T_wb.transform.translation.x = T_wb.translation().x();
            transformStamped_T_wb.transform.translation.y = T_wb.translation().y();
            transformStamped_T_wb.transform.translation.z = T_wb.translation().z();
            transformStamped_T_wb.transform.rotation.x = T_wb.so3().unit_quaternion().x();
            transformStamped_T_wb.transform.rotation.y = T_wb.so3().unit_quaternion().y();
            transformStamped_T_wb.transform.rotation.z = T_wb.so3().unit_quaternion().z();
            transformStamped_T_wb.transform.rotation.w = T_wb.so3().unit_quaternion().w();
            br.sendTransform(transformStamped_T_wb);

            transformStamped_T_wl.header.stamp = pose_Ptr->header.stamp;
            transformStamped_T_wl.transform.translation = transformStamped_T_wb.transform.translation;
            br.sendTransform(transformStamped_T_wl);
        }

        PointCloudP_ptr cloud (new PointCloudP);
        pcl::fromROSMsg (*pc_Ptr, *cloud);
        if(pc_Ptr->is_dense)
        {
        }else
        {//remove invalid pts
            vector<int> index;
            pcl::removeNaNFromPointCloud(*cloud,*cloud,index);
        }

        int pcsize = static_cast<int>(cloud->size());
        vector<Vec3> pc_eigen;
        if(pcsize>1000)
        {
            for(int i=0; i<1000; i++)
            {
                size_t rand_idx = static_cast<size_t>(rand() % pcsize);
                PointP pt = cloud->at(rand_idx);
                pc_eigen.push_back(Vec3(static_cast<double>(pt.x),
                                        static_cast<double>(pt.y),
                                        static_cast<double>(pt.z)));
            }
        }
        else
        {
            for(int i=0; i<pcsize; i++)
            {
                PointP pt = cloud->at(i);
                pc_eigen.push_back(Vec3(static_cast<double>(pt.x),
                                        static_cast<double>(pt.y),
                                        static_cast<double>(pt.z)));
            }
        }


        local_map->input_pc_pose(pc_eigen,T_wb);
        //cout << "publish local map" << endl;
        this->localmap_publisher->pub_localmap(local_map->visualization_cell_list,pose_Ptr->header.stamp);
        //cout << "publish local2global" << endl;
        l2g_pub->pub(local_map->l2g_msg_T_wl,local_map->l2g_msg_obs_pts_l,local_map->l2g_msg_miss_pts_l,pose_Ptr->header.stamp);
        //cout << "publish local2global end" << endl;
        //update_time.toc("update time");
        //local_map
    }


};//class LocalMapNodeletClass
}//namespace glmapping_ns

PLUGINLIB_EXPORT_CLASS(glmapping_ns::LocalMapNodeletClass, nodelet::Nodelet)


