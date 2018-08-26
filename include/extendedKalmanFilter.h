#include <ros/ros.h>
#include <armadillo>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>

#define PI 3.1415

using namespace arma;

class EKF
{
private:
	ros::NodeHandle n;
	ros::Publisher ekf_odom = n.adverstise<nav_msgs::Odometry>("/odom",1);
	ros::Subscriber sub = n.subscribe("/slam_out_pose",1,&EKF::slam_callback,this);
	ros::Publisher enco_pose = n.advertise<geometry_msgs::PoseWithCovariance>("/encoder_pose",1);
	ros::Subscriber right_encoder = n.subscribe("/right/tick",&EKF::right_encoder_callback,this);
	ros::Subscriber left_encoder = n.subscribe("/left/tick",&EKF::left_encoder_callback,this);
	float current_rencoder = 0.0,previous_rencoder = 0.0;
	float current_lencoder = 0.0,previous_lencoder = 0.0;
	
	//Holds the EKF estimated pose
	nav_msgs::Odometry odom_ekf;



	// Convariance matrices

	mat encoder_estimate_covariance; 

	mat process_covariance;


	//Jacobian matrices
	mat state_jacobian;

	mat process_jacobian;

	mat observation_jacobian;

	//Esimated and obervation pose matrices
	mat encoder_estimated_pose;
	mat observation_pose;
	

	//Constructor
	EKF();
	
	//Destructor
	~EKF();
	
	//Slam callback to get the observation vector
	void slam_callback(const geometry_msgs::PoseStamped &data);

		
	// Encoder callbacks

	void right_encoder_callback(const std_msgs::Float32& data);

	void left_encoder_callback(const std_msgs::Float32& data);



	// For running the ekg algorithm
	void efk_pose_calculation();
	
};
