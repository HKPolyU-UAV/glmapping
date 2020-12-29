#include "global2esdf3d.h"

Global2ESDF3DPatch::Global2ESDF3DPatch(ros::NodeHandle& nh, string topic_name, int buffersize)
{
    this->esdf_pub = nh.advertise<visualization_msgs::Marker>(topic_name, buffersize);
}

void Global2ESDF3DPatch::setGlobalMap(global_map_cartesian &map, string world_fram_name)
{
    this->map3d_nx = map.map_nx;
    this->map3d_ny = map.map_ny;
    this->map3d_nz = map.map_nz;
    this->map3d_dx = map.map_dx;
    this->map3d_dy = map.map_dy;
    this->map3d_dz = map.map_dz;

    visualized_layer = 0;
    cubes_array.header.frame_id  = world_fram_name;
    cubes_array.ns = "points";
    cubes_array.type = visualization_msgs::Marker::CUBE_LIST;
    cubes_array.action = visualization_msgs::Marker::ADD;
    cubes_array.pose.orientation.w =  1.0;
    cubes_array.scale.x = cubes_array.scale.y = map3d_dx;
    cubes_array.scale.z = 0.1;//a layer
    cubes_array.id = 0;
    cubes_array.points.clear();
    cubes_array.colors.clear();
    cout << "esdf map" << endl;

    esdf_cutoff_value = ESDF_BATCH_MAX_SERCH_RANGE*map3d_dx;
    for (int i=0; i<ESDF_BATCH_SIZE_NZ; i++)
    {
        MatrixXd esdf_layer;
        esdf_layer.resize(ESDF_BATCH_SIZE_NX_NY,ESDF_BATCH_SIZE_NX_NY);
        esdf_layer.fill(esdf_cutoff_value);
        esdf_map3d.push_back(esdf_layer);
    }
    cout << "esdf map init finished~" << endl;
}

Vec3 Global2ESDF3DPatch::esdf_cube_coler(double ratio)
{
    //we want to normalize ratio so that it fits in to 6 regions
    //where each region is 256 units long
    int normalized = int(ratio * 256 * 3);
    //find the distance to the start of the closest region
    int x = normalized % 256;
    int red = 0, grn = 0, blu = 0;
    switch(normalized / 256)
    {
        case 0: red = 255;      grn = x;        blu = 0;       break;//red
        case 1: red = 255 - x;  grn = 255;      blu = 0;       break;//yellow
        case 2: red = 0;        grn = 255;      blu = x;       break;//green
    }
    return Vec3(red/260.0,grn/260.0,blu/260.0);
}

void Global2ESDF3DPatch::pub_ESDF_3D_from_globalmap(global_map_cartesian &map, ros::Time stamp)
{

    for (size_t i=0; i<ESDF_BATCH_SIZE_NZ; i++)
    {
        esdf_map3d.at(i).fill(esdf_cutoff_value);
    }

    int idx_min_x, idx_min_y, idx_min_z;
    int idx_max_x, idx_max_y, idx_max_z;
    Vec3 translation_w_l = map.newest_T_wl.translation();
    Vec3I xyz_idx=map.xyz2xyzIdx(translation_w_l);

    idx_min_x = xyz_idx(0)-((ESDF_BATCH_SIZE_NX_NY-1)/2);
    idx_min_y = xyz_idx(1)-((ESDF_BATCH_SIZE_NX_NY-1)/2);
    idx_min_z = xyz_idx(2)-((ESDF_BATCH_SIZE_NZ-1)/2);
    idx_max_x = xyz_idx(0)+((ESDF_BATCH_SIZE_NX_NY-1)/2);
    idx_max_y = xyz_idx(1)+((ESDF_BATCH_SIZE_NX_NY-1)/2);
    idx_max_z = xyz_idx(2)+((ESDF_BATCH_SIZE_NZ-1)/2);

//    cout << "idx_min_x" << idx_min_x << endl;
//    cout << "idx_min_y" << idx_min_y << endl;
//    cout << "idx_min_z" << idx_min_z << endl;
//    cout << "idx_max_x" << idx_max_x << endl;
//    cout << "idx_max_y" << idx_max_y << endl;
//    cout << "idx_max_z" << idx_max_z << endl;


    for(auto cell:map.occupied_cell_idx_list)
    {
        int x = cell(0);
        int y = cell(1);
        int z = cell(2);
        if(     x>=idx_min_x && y>=idx_min_y && z>=idx_min_z &&
                x<=idx_max_x && y<=idx_max_y && z<=idx_max_z)
        {
            //calculate idx in batch
            int idx_x = x-idx_min_x;
            int idx_y = y-idx_min_y;
            int idx_z = z-idx_min_z;
            esdf_map3d.at(static_cast<size_t>(idx_z))(idx_x,idx_y)=ESDF_BATCH_OCCUPIED;
        }
    }
    //calculate the esdf for each layer
    for(size_t z=0; z<ESDF_BATCH_SIZE_NZ; z++)
    {
        for (int x = 0; x < ESDF_BATCH_SIZE_NX_NY; x++)
        {
            for (int y = 0; y < ESDF_BATCH_SIZE_NX_NY; y++)
            {
                if(esdf_map3d.at(z)(x,y) == ESDF_BATCH_OCCUPIED)
                {
                    for(int xx=x-ESDF_BATCH_MAX_SERCH_RANGE; xx<x+(ESDF_BATCH_MAX_SERCH_RANGE+1); xx++)
                    {
                        for(int yy=y-ESDF_BATCH_MAX_SERCH_RANGE; yy<y+(ESDF_BATCH_MAX_SERCH_RANGE+1); yy++)
                        {
                            if(     yy<0 || yy>(ESDF_BATCH_SIZE_NX_NY-1)
                                    || xx<0 || xx>(ESDF_BATCH_SIZE_NX_NY-1))
                            {//out of range
                                //cout << "xx yy out of range" << endl;
                                continue;
                            }
                            if(esdf_map3d.at(z)(xx,yy)==ESDF_BATCH_OCCUPIED)
                            {
                                continue;
                            }
                            double dis = sqrt(pow((abs(xx-x)*map3d_dx),2)+pow((abs(yy-y)*map3d_dy),2));
                            if(dis>esdf_cutoff_value) dis=esdf_cutoff_value;
                            if(esdf_map3d.at(z)(xx,yy)>dis)
                            {
                                esdf_map3d.at(z)(xx,yy)=dis;
                            }
                        }
                    }
                }
            }
        }
    }
    // integrate update to multiple layer
    MatrixXd increase_dz;
    increase_dz.resize(ESDF_BATCH_SIZE_NX_NY,ESDF_BATCH_SIZE_NX_NY);
    increase_dz.fill(map3d_dz);
    for(int loop=0; loop<ESDF_BATCH_MAX_SERCH_RANGE-1; loop++)
    {
        // go up
        for(size_t layer=0; layer<ESDF_BATCH_SIZE_NZ-1; layer++)
        {
            MatrixXd update_matrix = esdf_map3d.at(layer)+increase_dz;
            MatrixXd residual_matrix = update_matrix-esdf_map3d.at(layer+1);
            MatrixXb update_mask_matrix = residual_matrix.unaryExpr([](double d){ return d < 0.0; });
            for (int x = 0; x < ESDF_BATCH_SIZE_NX_NY; x++)
            {
                for (int y = 0; y < ESDF_BATCH_SIZE_NX_NY; y++)
                {
                    if(update_mask_matrix(x,y)==true)
                    {
                        esdf_map3d.at(layer+1)(x,y) = update_matrix(x,y);
                    }
                }
            }
        }
        // go down
        for(size_t layer=ESDF_BATCH_SIZE_NZ-1; layer>1; layer--)
        {
            MatrixXd update_matrix = esdf_map3d.at(layer)+increase_dz;
            MatrixXd residual_matrix = update_matrix-esdf_map3d.at(layer-1);
            MatrixXb update_mask_matrix = residual_matrix.unaryExpr([](double d){ return d < 0.0; });
            for (int x = 0; x < ESDF_BATCH_SIZE_NX_NY; x++)
            {
                for (int y = 0; y < ESDF_BATCH_SIZE_NX_NY; y++)
                {
                    if(update_mask_matrix(x,y)==true)
                    {
                        esdf_map3d.at(layer-1)(x,y) = update_matrix(x,y);
                    }
                }
            }
        }
    }
    mat2vismap_bias_x =0;
    mat2vismap_bias_y =0;
    this->mat2vismap_bias_x = -((static_cast<double>(map3d_nx)/2.0)-idx_min_x)*map3d_dx+0.5*map3d_dx;
    this->mat2vismap_bias_y = -((static_cast<double>(map3d_ny)/2.0)-idx_min_y)*map3d_dy+0.5*map3d_dy;
    cubes_array.points.clear();
    cubes_array.colors.clear();
    cubes_array.header.stamp = stamp;

    double max_esdf_range = esdf_cutoff_value*1.1;

    //visualized_layer = 1;
    double visulized_height;
    visulized_height = (idx_min_z+visualized_layer)*map3d_dz;

    MatrixXd esdf_vis_layer = esdf_map3d.at(visualized_layer);
    visualized_layer++;
    if(visualized_layer == ESDF_BATCH_SIZE_NZ) visualized_layer=0;

    for (int x = 0; x < ESDF_BATCH_SIZE_NX_NY; x++)
    {
        for (int y = 0; y < ESDF_BATCH_SIZE_NX_NY; y++)
        {
//            if(esdf_vis_layer(x,y) == ESDF_BATCH_UNKNOWN)
//            {//Unknown ->Nothing
//                geometry_msgs::Point point;
//                point.x = mat2vismap_bias_x+x*this->map3d_dx;
//                point.y = mat2vismap_bias_y+y*this->map3d_dy;
//                point.z = visulized_height;
//                this->cubes_array.points.push_back(point);
//                std_msgs::ColorRGBA color;
//                color.r= 0.0;
//                color.g= 0.8;
//                color.b= 0.0;
//                color.a= static_cast<float>(0.1);
//                this->cubes_array.colors.push_back(color);
//                continue;
//            }
            if(esdf_vis_layer(x,y) == ESDF_BATCH_OCCUPIED)
            {//Occupied ->BLACK
                geometry_msgs::Point point;
                point.x = mat2vismap_bias_x+x*this->map3d_dx;
                point.y = mat2vismap_bias_y+y*this->map3d_dy;
                point.z = visulized_height;
                this->cubes_array.points.push_back(point);
                std_msgs::ColorRGBA color;
                color.r= 0.0;
                color.g= 0.0;
                color.b= 0.0;
                color.a= static_cast<float>(0.9);
                this->cubes_array.colors.push_back(color);
            }else
            {//Color according to distance
                geometry_msgs::Point point;
                point.x = mat2vismap_bias_x+x*this->map3d_dx;
                point.y = mat2vismap_bias_y+y*this->map3d_dy;
                point.z = visulized_height;
                this->cubes_array.points.push_back(point);
                double ratio = esdf_vis_layer(x,y)/max_esdf_range;
                Vec3 rgb = this->esdf_cube_coler(ratio);
                std_msgs::ColorRGBA color;
                color.r= static_cast<float>(rgb(0));
                color.g= static_cast<float>(rgb(1));
                color.b= static_cast<float>(rgb(2));
                color.a= static_cast<float>(0.9);
                this->cubes_array.colors.push_back(color);
            }
        }
    }
    if(cubes_array.points.size()!=0)
    {
        this->esdf_pub.publish(cubes_array);
    }


}
