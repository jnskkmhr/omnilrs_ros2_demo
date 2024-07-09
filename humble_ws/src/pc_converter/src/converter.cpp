#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// Reference
// https://github.com/liuxiao916/isaac_sim_pointcloud_tool

// VLP-16 
int N_SCAN = 16;
int Horizon_SCAN = 1800;    

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

class Converter : public rclcpp::Node
{
    public:
        Converter(std::string node_name); 
    private:
        void lidar_handle(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg);
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPC_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPC_;
};


Converter::Converter(std::string node_name) : Node(node_name)
{
    subPC_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/pointcloud", 10, std::bind(&Converter::lidar_handle, this, std::placeholders::_1));
    pubPC_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points", 10);
}

void Converter::lidar_handle(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<PointXYZIRT>::Ptr pc_new(new pcl::PointCloud<PointXYZIRT>);
    pcl::fromROSMsg(*pc_msg, *pc);

    // to new pointcloud
    for (int point_id = 0; point_id < pc->points.size(); ++point_id) {

        PointXYZIRT new_point;
        new_point.x = pc->points[point_id].x;
        new_point.y = pc->points[point_id].y;
        new_point.z = pc->points[point_id].z;
        new_point.intensity = 0; 

        //16 ring. The range of index is 0~15. Up to Down.
        float ang_bottom = 15.0+0.1;
        float ang_res_y = 2;
        float verticalAngle = atan2(new_point.z, sqrt(new_point.x * new_point.x + new_point.y * new_point.y)) * 180 / M_PI;
        float rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
        
        new_point.ring = int(rowIdn);
        new_point.time = (point_id / N_SCAN)*0.1/Horizon_SCAN ;

        pc_new->points.push_back(new_point);
    }

    pc_new->is_dense = true; 

    // publish the new pointcloud
    sensor_msgs::msg::PointCloud2 pc_new_msg;
    pcl::toROSMsg(*pc_new, pc_new_msg);
    pc_new_msg.header = pc_msg->header;
    pubPC_->publish(pc_new_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Converter>("pc_converter");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}