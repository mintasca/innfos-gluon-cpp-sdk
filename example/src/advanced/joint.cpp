#include <thread>
#include <chrono>
#include <string.h>

#include "robot_interface.h"

int main(int argc, char *argv[])
{
	if(EstablishConnection())
	{
		return -1;
	}

	ActuatorGroup robot;
	if(robot.Initialization())
	{
		return -1;
	}

	int axis_num=robot.GetAxisNum();
	double target_joint[axis_num];
	memset(target_joint,0,sizeof(double)*axis_num);

	if(MoveToTargetJoint(&robot,target_joint))
	{
		return -1;
	}

	
	double inc_joint[axis_num];
	for (int i=0;i<axis_num;i++)
	{
		inc_joint[i] = -30*pi/180;
	}

	if(MoveJointIncremental(&robot,inc_joint))
	{
		return -1;
	}

	if(MoveToTargetJoint(&robot,target_joint))
	{
		return -1;
	}

	
	//robot.Shutdown();

	return 0;
}


