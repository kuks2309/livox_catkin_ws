#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

class Lidar3d_ROI
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_cloud3d_in;
    ros::Publisher  pub_cloud3d_roi;
    
    // velodyne처럼 pub
	ros::Publisher  pub_cloud3d_roi_velodyne;
	
    double x_min, x_max;
    double y_min, y_max;
    double z_min, z_max;

public:
    Lidar3d_ROI() : nh("~")
    {
        nh.param("x_min", x_min, 0.0);
        nh.param("x_max", x_max, 10.0);
        nh.param("y_min", y_min, -10.0);
        nh.param("y_max", y_max, 10.0);
        nh.param("z_min", z_min, -1.0);
        nh.param("z_max", z_max, 2.5);

        sub_cloud3d_in = nh.subscribe("/livox/lidar", 1, &Lidar3d_ROI::cloud3d_Callback, this);
        pub_cloud3d_roi = nh.advertise<sensor_msgs::PointCloud2>("/livox/livox_roi", 1);
        
        // 생성자에 퍼블리셔 추가
		pub_cloud3d_roi_velodyne = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);
    }

    void cloud3d_Callback(const sensor_msgs::PointCloud2ConstPtr& input_cloud)
    {
		// ROS 메시지를 PCL 포인트클라우드로 변환
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input_cloud, *cloud); 
        
        // ROI 필터 설정
        pcl::CropBox<pcl::PointXYZI> crop;
        crop.setInputCloud(cloud);
        crop.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
        crop.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));

		// ROI 적용
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>); 
        crop.filter(*cloud_filtered);  

		// 필터링된 포인트 클라우드를 ROS 메시지로 변환
        sensor_msgs::PointCloud2 output_cloud;
        pcl::toROSMsg(*cloud_filtered, output_cloud);
        output_cloud.header = input_cloud->header;
        output_cloud.header.stamp = ros::Time::now(); // 현재 시간으로 업데이트
        pub_cloud3d_roi.publish(output_cloud);

		// velodyne 필터링된 포인트 클라우드를 ROS 메시지로 변환
		sensor_msgs::PointCloud2 output_cloud2;
		pcl::toROSMsg(*cloud_filtered, output_cloud2);
		output_cloud2.header = input_cloud->header;
		output_cloud2.header.frame_id = "velodyne"; // 올바른 헤더 설정
		output_cloud2.header.stamp = ros::Time::now(); // 현재 시간으로 업데이트
		pub_cloud3d_roi_velodyne.publish(output_cloud2);

    }
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_3d_roi_node");
	Lidar3d_ROI lidar3d_roi; 
	ros::spin();

	return 0;
}
