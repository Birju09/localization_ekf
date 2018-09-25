#include "extendedKalmanFilter.h"
#define TOL 0.0001

EKF::EKF()
{
	ROS_INFO("EKF Estimation Started");
	//parameters
	ros::param::param("kx",kx,10000.0f);
	ros::param::param("ky",ky,10000.0f);
	ros::param::param("kth",kth,10000.0f);
	ros::param::param("stat_x",stat_x,0.1f);
	ros::param::param("stat_y",stat_y,0.1f);
	ros::param::param("stat_th",stat_th,0.1f);
	ros::param::param("wheel_dia",wheel_dia,0.1f);
	ros::param::param("wheel_base",wheel_base,0.28f);
	ros::param::param("encoder_pub_frame",encoder_pub_frame,std::string("encoder"));
	ros::param::param("ekf_pose_frame",ekf_pose_frame,std::string("ekf_pose"));
	ROS_INFO("PARMA sam = %f %f %f %f",kx,ky,kth,wheel_base);
	circum = wheel_dia * PI;
	encoder_estimate_covariance.eye(3,3);
	state_jacobian.eye(3,3);	
	observation_pose.zeros(3,1);
	encoder_estimated_pose.zeros(3,1);
	observation_jacobian.eye(3,3);
	estimate_covariance.eye(3,3);
	lidar_covariance.eye(3,3);
	ekf_pose.zeros(3,1);
	begin = ros::Time::now();
}

EKF::~EKF()
{
	ROS_INFO("EKF Shutting down");
}

void EKF::slam_callback(const geometry_msgs::PoseWithCovarianceStamped &data)
{
	observation_pose(0,0) = data.pose.pose.position.x;
	observation_pose(1,0) = data.pose.pose.position.y;
	double roll,pitch,yaw;
	tf::Quaternion quater;
	tf::quaternionMsgToTF(data.pose.pose.orientation, quater);
	tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);
	observation_pose(2,0) = yaw;
	//ROS_INFO("CHeck");
	lidar_covariance(0,0) = data.pose.covariance[0];
	lidar_covariance(1,1) = data.pose.covariance[7];
	lidar_covariance(2,2) = data.pose.covariance[35];
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
	if (fabs(delta_l - delta_r) < TOL ) { delta_l = delta_r;}
	else if ( fabs(fabs(delta_l) - fabs(delta_r)) < TOL && delta_l / delta_r < 0)
	{
		int sign_r,sign_l;
		delta_l >= 0 ? sign_l = 1 : sign_l = -1;
		delta_r >= 0 ? sign_r = 1 : sign_r = -1;
		delta_l = sign_l * fabs(delta_r);
	}
	previous_lencoder = current_lencoder;
	float theta_l = delta_l/wheel_base;
	float theta_r = delta_r/wheel_base;
	float dth = ((theta_l - theta_r)/2.0f);
	float velocity_r = delta_r / dt;
	float velocity_l = delta_l / dt;
	float delta_distance = (delta_r + delta_l)/2.0f;
	//ROS_INFO("Velocities are %f and %f",dth,delta_l);
	tf::Quaternion qz;
	//ROS_INFO("CHeck2");
	encoder_estimate_covariance(0,0) = kx * fabs(delta_r) + stat_x;
	encoder_estimate_covariance(1,1) = ky * fabs(delta_l) + stat_y;
	encoder_estimate_covariance(2,2) = kth * dth + stat_th;
	encoder_estimated_pose(2,0) = encoder_estimated_pose(2,0) + dth;
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
	static tf::TransformBroadcaster trans;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(encoder_estimated_pose(0,0),encoder_estimated_pose(1,0),0.0));
	transform.setRotation(qz);
	trans.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"map",encoder_pub_frame));
	
	state_jacobian(0,2) = (-delta_distance * temp_sin);
	state_jacobian(1,2) = (delta_distance * temp_cos);
	//process_jacobian(0,0) = (0.5 * temp_cos) + (delta_distance/2.0f) - (temp_sin/L
	//process_jacobian(0,1) = (0.5 * temp_cos) + (delta_distance/2.0f) + (temp_sin/L);
	//process_jacobian(1,0) = (0.5 * temp_sin) + (delta_distance * temp_cos * (0.5/L));
	//process_jacobian(1,1) = (0.5 * temp_sin) + (delta_distance * temp_cos * (-0.5/L));
		
}


void EKF::fused_pose()
{
	encoder_pose_calculation();
	geometry_msgs::PoseStamped odom_ekf;	
	estimate_covariance = ( state_jacobian * (estimate_covariance * state_jacobian.t()) ) + encoder_estimate_covariance;
	mat sk = (observation_jacobian * (estimate_covariance * observation_jacobian.t())) + lidar_covariance;
	//ROS_INFO("CHeck3");
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
	static tf::TransformBroadcaster trans;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(ekf_pose(0,0),ekf_pose(1,0),0.0));
	transform.setRotation(qz);
	trans.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"map",ekf_pose_frame));
	
}	
	
