#include "extendedKalmanFilter.cpp"

int main(int argc,char**argv)
{
	ros::init(argc,argv,"localization_node");
	EKF ekf_obj;
	ros::Rate r(300);
	while (ros::ok())
	{
		ros::spinOnce();
		ekf_obj.fused_pose();
		r.sleep();
	}
	return 0;
}
