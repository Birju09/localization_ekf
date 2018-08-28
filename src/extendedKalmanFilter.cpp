#include "extendedKalmanFilter.h"


EKF::EKF()
{
	ROS_INFO("EKF Estimation Started");
	
	
	encoder_estimate_covariance.zeros(2,2);
	process_covariance.zeros(3,3);
	state_jacobian.eye(3,3);
	process_jacobian.zeros(3,2);
	observation_pose.zeros(3,1);
	encoder_estimated_pose.zeros(3,1);
	observation_jacobian.eye(3,3);
	process_jacobian(2,0) = 1/L;
	process_jacobian(2,1) = -1/L;
	estimate_covariance.eye(3,3);
	lidar_covariance.eye(3,3);
	lidar_covariance(0,0) = 0.02;
	lidar_covariance(1,1) = 0.02;
	lidar_covariance(2,2) = 0.08;
	ekf_pose.zeros(3,1);
	begin = ros::Time::now();
}

EKF::~EKF()
{
	ROS_INFO("EKF Shutting down");
}

void EKF::slam_callback(const geometry_msgs::PoseStamped &data)
{
	observation_pose(0,0) = data.pose.position.x;
	observation_pose(1,0) = data.pose.position.y;
	observation_pose(2,0) = data.pose.orientation.z;
}


void EKF::right_encoder_callback(const std_msgs::Float64 &data)
{
	current_rencoder = data.data;
}

void EKF::left_encoder_callback(const std_msgs::Float64 &data)
{
	current_lencoder = data.data;
}

void EKF::encoder_pose_calculation()
{
	ros::spinOnce();
	float delta_r = (current_rencoder - previous_rencoder) * circum;
	//std::cout<< delta_r <<endl;
	previous_rencoder = current_rencoder;
	float dt = (ros::Time::now() - begin).toSec();
	float delta_l = (current_lencoder - previous_lencoder) * circum;
	previous_lencoder = current_lencoder;
	float theta_l = delta_l/L;
	float theta_r = delta_r/L;
	float velocity_r = delta_r / dt;
	float velocity_l = delta_l / dt;
	float delta_distance = (delta_r + delta_l)/2.0f;
	//ROS_INFO("Velocities are %f and %f",delta_r,delta_l);
	tf::Quaternion qz;
	encoder_estimate_covariance(0,0) = Kr * fabs(delta_r);
	encoder_estimate_covariance(1,1) = Kl * fabs(delta_l);
	encoder_estimated_pose(2,0) = encoder_estimated_pose(2,0) + ((theta_r - theta_l)/2.0f);
	qz = tf::createQuaternionFromRPY(0, 0, encoder_estimated_pose(2,0));
	float temp_cos = cos(encoder_estimated_pose(2,0));
	float temp_sin = sin(encoder_estimated_pose(2,0));
	encoder_estimated_pose(0,0) = encoder_estimated_pose(0,0) + (delta_distance * temp_cos);
	encoder_estimated_pose(1,0) = encoder_estimated_pose(1,0) + (delta_distance * temp_sin);
	encoder_pose.pose.position.x = encoder_estimated_pose(0,0);
	encoder_pose.pose.position.y = encoder_estimated_pose(1,0);
	encoder_pose.pose.orientation.z = qz[2];
	encoder_pose.pose.orientation.w = qz[3];
	enco_pose.publish(encoder_pose);
	
	state_jacobian(0,2) = (-delta_distance * temp_sin);
	state_jacobian(1,2) = (delta_distance * temp_cos);
	process_jacobian(0,0) = (0.5 * temp_cos) + (delta_distance/2.0f) - (temp_sin/L);
	process_jacobian(0,1) = (0.5 * temp_cos) + (delta_distance/2.0f) + (temp_sin/L);
	process_jacobian(1,0) = (0.5 * temp_sin) + (delta_distance * temp_cos * (0.5/L));
	process_jacobian(1,1) = (0.5 * temp_sin) + (delta_distance * temp_cos * (-0.5/L));
	
	
}


void EKF::fused_pose()
{
	encoder_pose_calculation();
	process_covariance = process_jacobian * (encoder_estimate_covariance * process_jacobian.t());
	estimate_covariance = ( state_jacobian * (estimate_covariance * state_jacobian.t()) ) + process_covariance;
	mat sk = (observation_jacobian * (estimate_covariance * observation_jacobian.t())) + lidar_covariance;
	mat kalman_gain = (estimate_covariance * (observation_jacobian.t() * sk.i()));
	//std::cout<<kalman_gain<<std::endl;
	ekf_pose = encoder_estimated_pose + (kalman_gain * (observation_pose - encoder_estimated_pose));
	estimate_covariance = (eye(3,3) - (kalman_gain * observation_jacobian)) * estimate_covariance;
	odom_ekf.pose.position.x = ekf_pose(0,0);
	odom_ekf.pose.position.y = ekf_pose(1,0);
	tf::Quaternion qz;
	qz = tf::createQuaternionFromRPY(0, 0, ekf_pose(2,0));
	odom_ekf.pose.orientation.z = qz[2];
	odom_ekf.pose.orientation.w = qz[3];
	odom_ekf.header.frame_id="map";
	ekf_odom.publish(odom_ekf);
}	
	
