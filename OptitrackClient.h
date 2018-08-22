#define NOMINMAX

#include <NatNetTypes.h>
#include <NatNetClient.h>

#include <stdio.h>
#include <tchar.h>
#include <iostream>
#include <conio.h>
#include <winsock2.h>
#include <boost\timer.hpp>

class OptitrackClientSample
{
public:
	OptitrackClientSample();
	~OptitrackClientSample();

	int Initialize();

	void setIPAddress(std::string ServerIPAddress, std::string MyIPAddress);

	void getPose(double *pose);
	int getFrameID();

private:

	int CreateClient(int iConnectionType);
	static void MessageHandler(int msgType, char* msg);
	static void DataHandler(sFrameOfMocapData* data, void* pUserData);

	
	int m_ConnectionType;
	int m_AnalogSamplesPerMocapFrame;

	sServerDescription m_ServerDescription;

	sDataDescriptions* m_pDataDefs;

	NatNetClient* m_pClient;
	char* m_pMyIPAddress;
	char* m_pServerIPAddress;

	int m_FrameID;
	double m_pose[7];
};

