#include "OptitrackClient.h"
#include "laserScan2PCL.h"

int main()
{
	double pose[7] = { 0, 0, 0, 0, 0, 0, 0 };

	OptitrackClientSample* client = new OptitrackClientSample();
	client->Initialize();

	LaserScan2PCL* laserScan2PCL = new LaserScan2PCL("pointcloud.pcd");

	int nFrame = client->getFrameID();

	while (1){
		if (nFrame != client->getFrameID()){
			nFrame = client->getFrameID();
			client->getPose(pose);
			//std::cout << "Positon:" << pose[0] << "   " << pose[1] << "   " << pose[2] << std::endl;
			laserScan2PCL->setTransform(pose);
			laserScan2PCL->spin();
		}
	}

	//getchar();
	return 0;
}