#include "global2esdf.h"

Global2ESDF::Global2ESDF(ros::NodeHandle& nh, string topic_name, int buffersize)
{
    this->esdf_pub = nh.advertise<visualization_msgs::Marker>(topic_name, buffersize);
}

void Global2ESDF::setGlobalMap(global_map_cartesian &map, string world_fram_name)
{
    this->map2d_nx = map.map_nx;
    this->map2d_ny = map.map_ny;
    this->map2d_dx = map.map_dx;
    this->map2d_dy = map.map_dy;
    this->map2mat_bias_x = (map2d_nx/2);
    this->map2mat_bias_y = (map2d_ny/2);
    this->mat2vismap_bias_x = -(static_cast<double>(map2d_nx)/2.0)*map2d_dx+0.5*map2d_dx;
    this->mat2vismap_bias_y = -(static_cast<double>(map2d_ny)/2.0)*map2d_dy+0.5*map2d_dy;

    cout << "map2d_nx" << map2d_nx << endl << "map2d_ny" << map2d_ny << endl;
    cout << "map2d_dx" << map2d_dx << endl << "map2d_dy" << map2d_dy << endl;
    cout << "map2mat_bias_x" << map2mat_bias_x << endl << "map2mat_bias_y" << map2mat_bias_y << endl;
    cout << "mat2vismap_bias_x" << mat2vismap_bias_x << endl << "mat2vismap_bias_x" << mat2vismap_bias_x << endl;

    cubes_array.header.frame_id  = world_fram_name;
    cubes_array.ns = "points";
    cubes_array.type = visualization_msgs::Marker::CUBE_LIST;
    cubes_array.action = visualization_msgs::Marker::ADD;
    cubes_array.pose.orientation.w =  1.0;
    cubes_array.scale.x = cubes_array.scale.y = map2d_dx;
    cubes_array.scale.z = 0.1;//a layer
    cubes_array.id = 0;
    cubes_array.points.clear();
    cubes_array.colors.clear();
    cout << "esdf map" << endl;

    this->esdf_map.resize(map2d_nx,map2d_ny);
    for (int x = 0; x < map2d_nx; x++)
    {
        for (int y = 0; y < map2d_ny; y++)
        {
            esdf_map(x,y)=ESDF_UNKNOWN;
        }
    }
    cout << "esdf map init finished~" << endl;
}

Vec3 Global2ESDF::esdf_cube_coler(double ratio)
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

void Global2ESDF::pub_ESDF_2D_from_globalmap(global_map_cartesian &map, ros::Time stamp)
{
    for(auto cell:map.occupied_cell_idx_list)
    {
//        int x = cell(0)+map2mat_bias_x;
//        int y = cell(1)+map2mat_bias_y;
        int x = cell(0);
        int y = cell(1);
        esdf_map(x,y) = ESDF_OCCUPIED;
        //cout << "add as cooupied" << endl;
    }
    for (int x = 0; x < map2d_nx; x++)
    {
        for (int y = 0; y < map2d_ny; y++)
        {
            if(esdf_map(x,y) == ESDF_OCCUPIED)
            {
                for(int xx=x-ESDF_SERCHRANGE; xx<x+(ESDF_SERCHRANGE+1); xx++)
                {
                    for(int yy=y-ESDF_SERCHRANGE; yy<y+(ESDF_SERCHRANGE+1); yy++)
                    {
                        if(yy<0 || yy>(map2d_ny-1) || xx<0 || xx>(map2d_nx-1))
                        {//out of range
                            //cout << "xx yy out of range" << endl;
                            continue;
                        }
                        if(esdf_map(xx,yy)==ESDF_OCCUPIED)
                        {
                            continue;
                        }
                        if(esdf_map(xx,yy)==ESDF_UNKNOWN)
                        {
                            //cout << sqrt(pow((abs(xx-x)*map2d_dx),2)+pow((abs(yy-y)*map2d_dy),2)) << endl;;
                            esdf_map(xx,yy) = sqrt(pow((abs(xx-x)*map2d_dx),2)+pow((abs(yy-y)*map2d_dy),2));
                            continue;
                        }
                        double dis=sqrt(pow((abs(xx-x)*map2d_dx),2)+pow((abs(yy-y)*map2d_dy),2));
                        if(esdf_map(xx,yy)>dis)
                        {
                            esdf_map(xx,yy)=dis;
                        }
                    }
                }
            }
        }
    }

    cubes_array.points.clear();
    cubes_array.colors.clear();
    cubes_array.header.stamp = stamp;

    double max_esdf_range=ESDF_SERCHRANGE*map2d_dx*1.5;
    for (int x = 0; x < map2d_nx; x++)
    {
        for (int y = 0; y < map2d_ny; y++)
        {
            if(esdf_map(x,y) == ESDF_UNKNOWN)
            {//Unknown ->Nothing
                geometry_msgs::Point point;
                point.x = mat2vismap_bias_x+x*this->map2d_dx;
                point.y = mat2vismap_bias_y+y*this->map2d_dy;
                point.z = -0.1;
                this->cubes_array.points.push_back(point);
                std_msgs::ColorRGBA color;
                color.r= 0.0;
                color.g= 0.8;
                color.b= 0.0;
                color.a= static_cast<float>(0.1);
                this->cubes_array.colors.push_back(color);
                continue;
            }
            if(esdf_map(x,y) == ESDF_OCCUPIED)
            {//Occupied ->BLACK
                geometry_msgs::Point point;
                point.x = mat2vismap_bias_x+x*this->map2d_dx;
                point.y = mat2vismap_bias_y+y*this->map2d_dy;
                point.z = -0.1;
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
                point.x = mat2vismap_bias_x+x*this->map2d_dx;
                point.y = mat2vismap_bias_y+y*this->map2d_dy;
                point.z = -0.1;
                this->cubes_array.points.push_back(point);
                double ratio = esdf_map(x,y)/max_esdf_range;
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
