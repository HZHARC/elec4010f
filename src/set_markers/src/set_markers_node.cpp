#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <string>
#include <geometry_msgs/PoseArray.h>

using namespace std;
using namespace ros;
ros::Publisher marker_pub;
uint32_t shape = visualization_msgs::Marker::SPHERE;

void callback(const geometry_msgs::PoseArray& poseArray)
{
  vector<visualization_msgs::Marker> markers;
  markers.resize(poseArray.poses.size());
  for (int i=0; i<markers.size();++i){
    markers[i].header.frame_id = "/camera_rgb_optical_frame";
    markers[i].header.stamp = ros::Time::now();
    markers[i].ns = "faces";
    markers[i].id = i+1;
    markers[i].type = shape;
    markers[i].action = visualization_msgs::Marker::ADD;
    markers[i].pose = poseArray.poses[i];
    double scale = 0.2;
    markers[i].scale.x = scale;
    markers[i].scale.y = scale;
    markers[i].scale.z = scale;
    markers[i].color.r = 0.0f;
    markers[i].color.g = 1.0f;
    markers[i].color.b = 0.0f;
    markers[i].color.a = 1.0;
    markers[i].lifetime = ros::Duration();
    marker_pub.publish(markers[i]);
  } 
}
// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "set_markers_node");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber subToFacePoses = n.subscribe("target_location",1,callback);
  ros::spin();
}


