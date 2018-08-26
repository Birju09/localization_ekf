#include "extendedKalmanFilter.h"


EKF::EKF()
{
	ROS_INFO("EKF Estimation Started");
	observation_jacobian.eye(3,3);	
}

EKF::~EKF()
{
	ROS_INFO("EKF Shutting down");
}

void EKF::slam_callback(const geometry_msgs::PoseStamped &data)
{
	observation_pose(0,0) = data.pose.position.x;
	observation_pose(0,1) = data.pose.position.y;
	observation_pose(0,2) = data.pose.orientation.z;
}


void ekf_pose_calculcation()
{
	
