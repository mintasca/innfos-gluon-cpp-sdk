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

	robot.Shutdown();

	std::cout << "Done" << std::endl;

	return 0;
}



