#include <ros/ros.h>
#include <kalman/DoorUpdate.h>
#include <kalman/KalmanUpdate.h>

/*
	KalmanUpdate: c++ function called to update the kalman filter

	args: 	req.u (2-dimensional state estimate vector [u udot] at time t)
			req.sigma (4-element vector with sigma matrix:
					[ sigma[0] sigma[1]
					  sigma[2] sigma[3]]
			req.z (measurement value)

			res.u (2-dimensional vector with updated state at time t+1)
			req.sigma (4-element vector holding sigma matrix (same layout as above))

	returns: true if it works
*/


bool KalmanUpdate( 	kalman::KalmanUpdate::Request &req,
					kalman::KalmanUpdate::Response &res )
{
	res.u = req.u;
	res.sigma = req.sigma;
	return true;
}

/*
	DoorUpdate: c++ function called to update the doors estimate filter

	args: 	req.u (2-dimensional state estimate vector [u udot] at time t)
			req.sigma (4-element vector with sigma matrix:
					[ sigma[0] sigma[1]
					  sigma[2] sigma[3]]
			req.d (boolean, true if a door is currently detected)
			req.door_dist: (10-element vector door_dist[i] = the probability (0..1) that there is a door at position i)

			res.door_dist: (10-element vector door_dist[i] = the probability (0..1) that there is a door at position i)
	returns: true if it works
*/

bool DoorUpdate( 	kalman::DoorUpdate::Request &req,
					kalman::DoorUpdate::Response &res )
{
	res.door_dist = req.door_dist;
	return true;
}

/* don't need to touch any of the code below, only the functions above */

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "kalman_srv");
	ros::NodeHandle nh;

	ros::ServiceServer kservice = nh.advertiseService("kalman_update", KalmanUpdate);
	ros::ServiceServer dservice = nh.advertiseService("door_update", DoorUpdate);

	ROS_INFO( "c++ code is ready to run");

	ros::spin();

	return 0;
}