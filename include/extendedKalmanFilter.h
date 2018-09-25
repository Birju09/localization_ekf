#include <ros/ros.h>
#include <armadillo>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>


constexpr float PI=3.1415;
float circum=0.0f;

using namespace arma;



class EKF
{
private:
	ros::NodeHandle n;
	ros::Publisher ekf_odom = n.advertise<geometry_msgs::PoseStamped>("/odom",1);
	ros::Subscriber sub = n.subscribe("/poseupdate",1,&EKF::slam_callback,this);
	ros::Publisher enco_pose = n.advertise<geometry_msgs::PoseWithCovariance>("/encoder_pose",1);
	ros::Subscriber right_encoder = n.subscribe("/right/eTick",1,&EKF::right_encoder_callback,this);
	ros::Subscriber left_encoder = n.subscribe("/left/eTick",1,&EKF::left_encoder_callback,this);
	
	//ros::Subscriber vel_pose = n.subscribe("/cmd_vel",1,&EKF::vel_callback,this);
	float current_rencoder = 0.0,previous_rencoder = 0.0;
	float current_lencoder = 0.0,previous_lencoder = 0.0;
	//geometry_msgs::Twist vel;
	ros::Time begin;
	//Holds the EKF estimated pose

	geometry_msgs::PoseWithCovariance encoder_pose;

	// Convariance matrices

	mat estimate_covariance;

	mat encoder_estimate_covariance; 

	mat lidar_covariance;

	//Jacobian matrices
	mat state_jacobian;

	mat observation_jacobian;

	//Esimated and obervation pose matrices
	mat encoder_estimated_pose;
	mat observation_pose;
	mat ekf_pose;

	
	
public:

	//parameters
	float kx,ky,kth;
	float stat_x,stat_y,stat_th;
	float wheel_dia;
	float wheel_base;
	
	//parameter frames
	std::string encoder_pub_frame;
	std::string ekf_pose_frame;

	//Constructor
	EKF();
	
	//Destructor
	~EKF();
	
	//Slam callback to get the observation vector
	void slam_callback(const geometry_msgs::PoseWithCovarianceStamped &data);

	//void vel_callback(const geometry_msgs::Twist &data);	
	// Encoder callbacks

	void right_encoder_callback(const std_msgs::Float64& data);
	void left_encoder_callback(const std_msgs::Float64& data);

	void encoder_pose_calculation();

	// For running the ekg algorithm
	void fused_pose();
	
};
