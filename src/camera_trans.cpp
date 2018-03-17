#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
void pointCloudCb(const PointCloud::ConstPtr&  cloud){
        pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
}
int main(int argc,char** argv)
{
  ros::init(argc,argv,"camera_transform");
  ros::NodeHandle nh;
ros::Rate rate(3.0);
    tf::TransformListener *tf_listener;
  tf_listener = new tf::TransformListener;
    std::cout<<"tf listener between kinect2 and table_top happens"<<std::endl;
  tf::StampedTransform transform;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/table_top_points", 1, pointCloudCb);
while (nh.ok()){
   tf_listener->waitForTransform("kinect2_rgb_optical_frame","/table_top", ros::Time::now(),ros::Duration(5.0));
    tf_listener->lookupTransform ("kinect2_rgb_optical_frame","/table_top",ros::Time(0), transform);

     std::cout<<"normalized rotation: "<<transform.getRotation().normalized ().getX () << transform.getRotation().normalized ().getY ()<<transform.getRotation().normalized ().getZ ()<<transform.getRotation().normalized ().getW ()<<std::endl;
tf::Quaternion ass(0.638, 0.406, -0.347, 0.555);
tf::Matrix3x3 mmm_echo(transform.getRotation());
Eigen::Matrix3d mm1_echo;
tf::matrixTFToEigen(mmm_echo,mm1_echo);
std::cout<<"eigen matrix: "<<mm1_echo<<std::endl;

tf::Matrix3x3 mmm(transform.getRotation());
Eigen::Matrix3d mm1;
tf::matrixTFToEigen(mmm,mm1);
std::cout<<"eigen matrix: "<<mm1<<std::endl;
     std::cout<<"traslation: "<<transform.getOrigin().getX () <<" "<< transform.getOrigin().getY ()<<" "<<transform.getOrigin().getZ () <<" "<<std::endl;

 rate.sleep();
}
ros::spin();
  return 0;
}
