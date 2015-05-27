#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

using namespace std;
using namespace ros;

int width = 640;
ros::Publisher pub;

class LocationService {
	vector<int> idx;
public:
	void initIndices();
	vector<int>  getInd() const { return idx; } 
	void setInd(vector<int> newInd) {idx = newInd; }
	void cloudCallback(const pcl::PCLPointCloud2ConstPtr& inputCloud);
	void indCallback(const std_msgs::Int32MultiArray& idx);
};

void LocationService::cloudCallback(const pcl::PCLPointCloud2ConstPtr& inputCloud)
{
	if (idx.size()==0) return;
	
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
  	pcl::fromPCLPointCloud2(*inputCloud, cloud);
  	geometry_msgs::PoseArray newPoseArray;
  	newPoseArray.header.frame_id = "/camera_rgb_optical_frame";
    newPoseArray.header.stamp = ros::Time::now();
    newPoseArray.poses.resize(idx.size());

    for (int i=0;i<idx.size();++i){
    int index = idx[i];
	newPoseArray.poses[i].orientation.x = 0.0;
	newPoseArray.poses[i].orientation.y = 0.0;
	newPoseArray.poses[i].orientation.z = 0.0;
	newPoseArray.poses[i].orientation.w = 1.0;
    newPoseArray.poses[i].position.x = cloud.points[index].x;
    newPoseArray.poses[i].position.y = cloud.points[index].y;
    newPoseArray.poses[i].position.z = cloud.points[index].z;
	}
	pub.publish(newPoseArray);
}

void LocationService::initIndices()
{
	idx.resize(4);
	idx[0] = 0;
	idx[1] = (240-1)*640+0;
	idx[2] = (240-1)*640+320;
	idx[3] = (240-1)*640+630;
}

void LocationService::indCallback(const std_msgs::Int32MultiArray& newInd)
{

	idx = newInd.data;
}

int main (int argc, char** argv)
{	
	ros::init (argc, argv, "get_location_node");
	ros::NodeHandle nh;
	LocationService location_service;

    ros::Subscriber depthListener = nh.subscribe("camera/depth/points", 1, &LocationService::cloudCallback,&location_service);
    ros::Subscriber indListener = nh.subscribe ("face_idxs", 1, &LocationService::indCallback,&location_service);
    pub = nh.advertise<geometry_msgs::PoseArray> ("target_location", 1);
   	ros::spin();
} 