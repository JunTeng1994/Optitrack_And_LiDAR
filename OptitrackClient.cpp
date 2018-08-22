#include "OptitrackClient.h"

using namespace std;

OptitrackClientSample::OptitrackClientSample() :
m_pClient(NULL),
m_pMyIPAddress("127.0.0.1"),
m_pServerIPAddress("127.0.0.1"),
m_ConnectionType(ConnectionType_Multicast),
m_AnalogSamplesPerMocapFrame(0),
m_ServerDescription(),
m_pDataDefs(NULL),
m_FrameID(0)
{
	for (int i = 0; i < 7; i++)
	{
		m_pose[i] = 0;
	}
	memset(&m_ServerDescription, 0, sizeof(sServerDescription));
}

OptitrackClientSample::~OptitrackClientSample()
{
	m_pClient->Uninitialize();
}

void OptitrackClientSample::setIPAddress(std::string ServerIPAddress, std::string MyIPAddress)
{
	strcpy(m_pServerIPAddress, ServerIPAddress.c_str());
	strcpy(m_pMyIPAddress, MyIPAddress.c_str());
}

void OptitrackClientSample::getPose(double *pose)
{
	for (int i = 0; i < 7; i++)
	{
		pose[i] = m_pose[i];
	}
}

int OptitrackClientSample::getFrameID()
{
	return m_FrameID;
}

void OptitrackClientSample::MessageHandler(int msgType, char* msg)
{
	cout << endl << msg << endl;
}

void OptitrackClientSample::DataHandler(sFrameOfMocapData* data, void* pUserData)
{
	OptitrackClientSample* pClientSample = (OptitrackClientSample*)pUserData;

	pClientSample->m_FrameID = data->iFrame;
	pClientSample->m_pose[0] = data->RigidBodies[0].x;
	pClientSample->m_pose[1] = data->RigidBodies[0].y;
	pClientSample->m_pose[2] = data->RigidBodies[0].z;
	pClientSample->m_pose[3] = data->RigidBodies[0].qx;
	pClientSample->m_pose[4] = data->RigidBodies[0].qy;
	pClientSample->m_pose[5] = data->RigidBodies[0].qz;
	pClientSample->m_pose[6] = data->RigidBodies[0].qw;
}

int OptitrackClientSample::CreateClient(int iConnectionType)
{
	if (m_pClient)
	{
		m_pClient->Uninitialize();
		delete m_pClient;
	}

	m_pClient = new NatNetClient(iConnectionType);

	m_pClient->SetVerbosityLevel(Verbosity_Warning);
	m_pClient->SetMessageCallback(&OptitrackClientSample::MessageHandler);
	m_pClient->SetDataCallback(&OptitrackClientSample::DataHandler, this);

	unsigned char ver[4];
	m_pClient->NatNetVersion(ver);
	printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

	int retCode = m_pClient->Initialize(m_pMyIPAddress, m_pServerIPAddress);

	if (retCode != ErrorCode_OK)
	{
		cout << "Unable to connect to server. Error code: " << retCode << ". Exiting" << endl;
		return ErrorCode_Internal;
	}
	else
	{
		void* pResult;
		int ret = 0;
		int nBytes = 0;
		ret = m_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
		if (ret == ErrorCode_OK)
		{
			m_AnalogSamplesPerMocapFrame = *(int*)pResult;
			cout << "Analog Samples Per Mocap Frame: " << m_AnalogSamplesPerMocapFrame << endl;
		}

		//print server info
		m_pClient->GetServerDescription(&m_ServerDescription);
		if (!m_ServerDescription.HostPresent)
		{
			cout << "Unable to connect to server. Host not present. Exiting..." << endl;
			return 1;
		}
		cout << "[SampleClient] Server application info:" << endl;
		printf("Application: %s (ver. %d.%d.%d.%d)\n", m_ServerDescription.szHostApp, m_ServerDescription.HostAppVersion[0],
            m_ServerDescription.HostAppVersion[1],m_ServerDescription.HostAppVersion[2],m_ServerDescription.HostAppVersion[3]);
		printf("NatNet Version: %d.%d.%d.%d\n", m_ServerDescription.NatNetVersion[0], m_ServerDescription.NatNetVersion[1],
			m_ServerDescription.NatNetVersion[2], m_ServerDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", m_pMyIPAddress);
        printf("Server IP:%s\n", m_pServerIPAddress);
        printf("Server Name:%s\n\n", m_ServerDescription.szHostComputerName);
	}

	return ErrorCode_OK;
}



int OptitrackClientSample::Initialize()
{
	int iResult;

	iResult = CreateClient(m_ConnectionType);
	if (iResult != ErrorCode_OK)
	{
		cout << "Error initializing client. See log for details. Exiting..." << endl;
		return 1;
	}
	else
	{
		cout << "Client initialized and ready." << endl;
	}

	cout << "[SampleClient] Sending Test Request" << endl;
	void* response;
	int nBytes;
	iResult = m_pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (iResult == ErrorCode_OK)
	{
		printf("[SampleClient] Received: %s", (char*)response);
	}

	//Retrieve Data Descriptions from server.
	cout << endl << endl << "[SampleClient] Requesting Data Descriptions..." << endl;
	int nBodies = m_pClient->GetDataDescriptions(&m_pDataDefs);
	if (!m_pDataDefs)
	{
		cout << "[SampleClient] Unable to retrieve Data Descriptions." << endl;
	}
	else
	{
		cout << "[SampleClient] Received " << m_pDataDefs->nDataDescriptions << " Data Descriptions: " << endl;
		for (int i = 0; i < m_pDataDefs->nDataDescriptions; i++)
		{
			printf("Data Description # %d (type=%d)\n", i, m_pDataDefs->arrDataDescriptions[i].type);
			if (m_pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
			{
				sMarkerSetDescription* pMS = m_pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
				cout << "MarkerSet Name: " << pMS->szName << endl;
				for (int j = 0; j < pMS->nMarkers; j++)
					cout << pMS->szMarkerNames[i] << endl;
			}
			else if (m_pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
			{
				sRigidBodyDescription* pRB = m_pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
				cout << "RigidBody Name: " << pRB->szName << endl;
				cout << "RigidBody ID: " << pRB->ID << endl;
				cout << "RigidBody Parent ID: " << pRB->parentID << endl;
				printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
			}
			else if (m_pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
            {
                // Skeleton
                sSkeletonDescription* pSK = m_pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
                printf("Skeleton Name : %s\n", pSK->szName);
                printf("Skeleton ID : %d\n", pSK->skeletonID);
                printf("RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
                for(int j=0; j < pSK->nRigidBodies; j++)
                {
                    sRigidBodyDescription* pRB = &pSK->RigidBodies[j];
                    printf("  RigidBody Name : %s\n", pRB->szName);
                    printf("  RigidBody ID : %d\n", pRB->ID);
                    printf("  RigidBody Parent ID : %d\n", pRB->parentID);
                    printf("  Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
                }
            }
            else if(m_pDataDefs->arrDataDescriptions[i].type == Descriptor_ForcePlate)
            {
                // Force Plate
                sForcePlateDescription* pFP = m_pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription;
                printf("Force Plate ID : %d\n", pFP->ID);
                printf("Force Plate Serial : %s\n", pFP->strSerialNo);
                printf("Force Plate Width : %3.2f\n", pFP->fWidth);
                printf("Force Plate Length : %3.2f\n", pFP->fLength);
                printf("Force Plate Electrical Center Offset (%3.3f, %3.3f, %3.3f)\n", pFP->fOriginX,pFP->fOriginY, pFP->fOriginZ);
                for(int iCorner=0; iCorner<4; iCorner++)
                    printf("Force Plate Corner %d : (%3.4f, %3.4f, %3.4f)\n", iCorner, pFP->fCorners[iCorner][0],pFP->fCorners[iCorner][1],pFP->fCorners[iCorner][2]);
                printf("Force Plate Type : %d\n", pFP->iPlateType);
                printf("Force Plate Data Type : %d\n", pFP->iChannelDataType);
                printf("Force Plate Channel Count : %d\n", pFP->nChannels);
                for(int iChannel=0; iChannel<pFP->nChannels; iChannel++)
                    printf("\tChannel %d : %s\n", iChannel, pFP->szChannelNames[iChannel]);
            }
            else
            {
                printf("Unknown data type.");
                // Unknown
            }
		}
	}		
}