//#include <jni.h>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/trigonometric.hpp>
#include <glm/exponential.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <math.h>
#include <string>

#ifdef WIN32
#include <stdio.h>
#include <tchar.h>
#include <windows.h>
#else // LINUX
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#endif
#include "vrtrackplugin.h"

#define MAX_UDP_RCV_LEN 1500
class CTracking
{
public:
#ifdef WIN32
	DWORD   dwThreadId;
	HANDLE  hThread;
#else
	pthread_t hThread;
#endif
	int m_fRun;
	glm::vec3 newPos[3];
	glm::vec3 startPos[3];
	glm::quat m_qatOrient;


	int sockfd;
	int remote_portno;
	int n;
	struct sockaddr_in serveraddr;
	struct sockaddr_in myaddr;
	int local_portno;

	int serverlen;
	char buf[MAX_UDP_RCV_LEN];

	float period;

	float A_offsetX, A_offsetY, A_offsetZ;

    CTracking()
	{
		newPos[0] = glm::vec3(0);
		startPos[0] = glm::vec3(3, 0, 0);
		startPos[1] = glm::vec3(0, 0, 0);
		startPos[2] = glm::vec3(1.5, -2.667, 0);

		A_offsetX = 0.0;
		A_offsetY = 0.0;
		A_offsetZ = 0.0;

		period = 1111122;
		remote_portno = 0;
		local_portno = 59427;
		memset(&serveraddr, 0x00, sizeof(serveraddr));
		memset(&myaddr, 0x00, sizeof(myaddr));
	}

	// Use this for initialization
	int Start() {
		sockfd = socket(AF_INET, SOCK_DGRAM, 0);
		if (sockfd < 0) {
			return -2;
		}
		serveraddr.sin_family = AF_INET;
		serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
		serveraddr.sin_port = htons(remote_portno);
		serverlen = sizeof(serveraddr);

		myaddr.sin_family = AF_INET;
		myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
		myaddr.sin_port = htons(local_portno);

		if (bind(sockfd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
			printf("bind failed");
			return -3;
		}
		//startPos = this.transform.position; //test();
		printf("Client Started");
		m_fRun = 1;
#ifdef WIN32
	hThread = CreateThread( NULL, 0, ReceiveData, this, 0, &dwThreadId);
#else
		if (pthread_create(&hThread, NULL, ReceiveData, NULL)) {

		}
#endif
		return 0;
	}

    int getData(char *data, int nLen)
    {
    	int bytesRecvd = recvfrom(sockfd, data, nLen, 0, (sockaddr*)&serveraddr, &serverlen);
    	return bytesRecvd;
    }

#ifdef WIN32
static DWORD WINAPI ReceiveData( LPVOID pArg )
#else
	static void* ReceiveData(void *pArg) 
#endif
{
		CTracking *pTracking = (CTracking*) pArg;
		char data[256];
		glm::vec4 vecDist(3, 3, 3, 0x00);
		bool altitudeReady = false;
		bool azthmusReady = false;
		glm::vec4 vecTheta;
		glm::vec4 vecBeta;

		while (pTracking->m_fRun) {
			int n = pTracking->getData(data, 256);
			if (n < 0) {
#ifdef WIN32
				printf("Socket error %d\n", WSAGetLastError());
#else
				printf("Socket error %d\n", errno);
#endif
				break;
			} else if (n== 0){
				printf("Socket error no data\n");
				continue;
			}

			if (data[1] == 'x') {
				vecBeta = pTracking->parseSensorDataSweep(data);
				altitudeReady = true;
			}
			else if (data[1] == 'y') {
				vecTheta = pTracking->parseSensorDataSweep(data);
				azthmusReady = true;
			}

			if (altitudeReady && azthmusReady) {

				glm::vec4 vecAngl = pTracking->extractAngles(vecBeta, vecTheta);
				glm::vec4 vecX = pTracking->NewtonsNewMethod(vecDist, vecAngl);
				pTracking->updatePositions(vecX, vecBeta, vecTheta);
				pTracking->updateOrientation(pTracking->newPos, pTracking->startPos);
				altitudeReady = false;
				azthmusReady = false;
			}
		}
		return 0;
	}

    void parseSensorData(char *data)
    {
 
    }
#define NUM_WIDTH 9
#define NUM_POS_1 3
#define NUM_POS_2 13
#define NUM_POS_3 23

    glm::vec4  parseSensorDataSweep(char *data)
    {
		glm::vec4 vecAngl;
        float tempB1, tempB2, tempB3;
        char szFloatNum[16]={0};
        
		memcpy(szFloatNum, data+NUM_POS_1, NUM_WIDTH);
		vecAngl[0] = atof(szFloatNum) / period * 360; // (float.Parse(splitArray[1]) / period) * 360;
       
		memcpy(szFloatNum, data+NUM_POS_2, NUM_WIDTH);
        vecAngl[1] =  atof(szFloatNum) / period * 360;
        
		memcpy(szFloatNum, data+NUM_POS_3, NUM_WIDTH);
        vecAngl[2] = atof(szFloatNum) / period * 360;

        //print("M_B1: " + tempB1 + " M_B2: " + tempB2 + "M_B3: " + tempB3);
        vecAngl[0] = glm::radians(vecAngl[0]);//tempB1 * Mathf.Deg2Rad;
        vecAngl[1] = glm::radians(vecAngl[1]);//tempB2 * Mathf.Deg2Rad;
        vecAngl[2] = glm::radians(vecAngl[2]);;//tempB3 * Mathf.Deg2Rad;
		return vecAngl;
    }

    glm::vec4 extractAngles(glm::vec4 vecBeta, glm::vec4 vecTheta)
    {
		glm::vec4 vecAngl;
		float B1 = vecBeta[0];
		float B2 = vecBeta[1];
		float B3 = vecBeta[2];

		float theta1 = vecTheta[0];
		float theta2 = vecTheta[1];
		float theta3 = vecTheta[2];

        vecAngl[0] = (glm::sin(B1) * glm::cos(theta1) * glm::sin(B2) * glm::cos(theta2) + (glm::sin(B1) * glm::sin(theta1) * glm::sin(B2) * glm::sin(theta2) + glm::cos(B1) * glm::cos(B2)));
        vecAngl[1] = (glm::sin(B2) * glm::cos(theta2) * glm::sin(B3) * glm::cos(theta3) + (glm::sin(B2) * glm::sin(theta2) * glm::sin(B3) * glm::sin(theta3) + glm::cos(B2) * glm::cos(B3)));
        vecAngl[2] = (glm::sin(B1) * glm::cos(theta1) * glm::sin(B3) * glm::cos(theta3) + (glm::sin(B1) * glm::sin(theta1) * glm::sin(B3) * glm::sin(theta3) + glm::cos(B1) * glm::cos(B3)));
		return vecAngl;
    }

	glm::mat4 getJacobian(glm::vec4 vecDist, glm::vec4 vecAngl, glm::vec4 vecX)
	{
		float m_distAB = vecDist[0];
		float m_distBC = vecDist[1];
		float m_distAC = vecDist[2];
		float cos12 = vecAngl[0];
		float cos23 = vecAngl[1];
		float cos13 = vecAngl[2];

		float ra = vecX[0];
		float rb = vecX[1]; 
		float rc = vecX[2];

		glm::mat4 matrixA;// = Matrix4x4.identity;

		matrixA[0][0] = ((2 * ra) - (2 * rb * cos12));
		matrixA[1][0] = ((2 * rb) - (2 * ra * cos12));
		matrixA[2][0] = 0;

		matrixA[0][1] = 0;
		matrixA[1][1] = ((2 * rb) - (2 * rc * cos23));
		matrixA[2][1] = ((2 * rc) - (2 * rb * cos23));
		
		matrixA[0][2] = ((2 * ra) - (2 * rc * cos13));
		matrixA[1][2] = 0; 
		matrixA[2][2] = ((2 * rc) - (2 * ra * cos13));
		return matrixA;
	}
	
	/*
	**
	*/
	glm::vec4 evalF(glm::vec4 vecDist, glm::vec4 vecAngl, glm::vec4 vecX)
	{
		float ra = vecX[0];
		float rb = vecX[1]; 
		float rc = vecX[2];
		float m_distAB = vecDist[0];
		float m_distBC = vecDist[1];
		float m_distAC = vecDist[2];
		float cos12 = vecAngl[0];
		float cos23 = vecAngl[1];
		float cos13 = vecAngl[2];

		glm::vec4 vecB(0.0f);// = Matrix4x4.identity;
		vecB[0] = (glm::pow(ra, 2) + glm::pow(rb, 2) - (2 * ra * rb * cos12) - glm::pow(m_distAB, 2));
        vecB[1] = (glm::pow(rb, 2) + glm::pow(rc, 2) - (2 * rb * rc * cos23) - glm::pow(m_distBC, 2));
        vecB[2] = (glm::pow(ra, 2) + glm::pow(rc, 2) - (2 * ra * rc * cos13) - glm::pow(m_distAC, 2));

		return vecB;
	}

     glm::vec4 NewtonsNewMethod(glm::vec4 vecDist, glm::vec4 vecAngl) {

		float m_distAB = vecDist[0];
		float m_distBC = vecDist[1]; 
		float m_distAC = vecDist[2];
		float m_cos12 = vecAngl[0];
		float m_cos23 = vecAngl[1];
		float m_cos13 = vecAngl[2];

		float gRA, gRB, gRC;
		glm::vec4 vecTmp(24,24,24,0);
		glm::vec4 vecX = vecTmp;
        for (int i = 0; i < 50; i++) {
			glm::vec4 vectorB = evalF(vecDist, vecAngl, vecX);
			glm::mat4 matrixA = getJacobian(vecDist, vecAngl, vecX);// = Matrix4x4.identity;

			vecX = vecTmp - glm::inverse(matrixA) * vectorB;

			if (vecTmp[0] == vecX[0] && vecTmp[1] == vecX[1] && vecTmp[2]== vecX[2]) {
				//printf("Problem Solved!"); 
				break;
			} else {
				vecTmp = vecX;
			}
        }
		return vecX;
    }

 
    void updatePositions(glm::vec4 vecX, glm::vec4 vecBeta, glm::vec4 vecTheta)
    {
		static int count = 0;
		for (int i=0; i < 3; i++) {
			float y = glm::abs(vecX[i]) * glm::sin(vecBeta[i]) * glm::cos(vecTheta[i]);
			float z = glm::abs(vecX[i]) * glm::sin(vecBeta[i]) * glm::sin(vecTheta[i]);
			float x = glm::abs(vecX[i]) * glm::cos(vecBeta[i]) /** glm::sin(vecTheta[i])*/;///Verify
			newPos[i] = glm::vec3(x, y, z);
			if(count++ % 60 == 0) {
				printf("Pos:x=%3.6f y=%3.6f z=%3.6f\n", x,y,z);
			}
		}
    }

	glm::quat RotationBetweenVectors(glm::vec3 start, glm::vec3 dest){
		start = normalize(start);
		dest = normalize(dest);

		float cosTheta = dot(start, dest);
		glm::vec3 rotationAxis;
/*
		if (cosTheta < -1 + 0.001f){
			// special case when vectors in opposite directions:
			// there is no "ideal" rotation axis
			// So guess one; any will do as long as it's perpendicular to start
			rotationAxis = glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), start);
			if (glm::length2(rotationAxis) < 0.01 ) // bad luck, they were parallel, try again!
				rotationAxis = glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), start);

			rotationAxis = normalize(rotationAxis);
			return glm::angleAxis(180.0f, rotationAxis);
		}
*/
		rotationAxis = glm::cross(start, dest);

		float s = sqrt( (1+cosTheta)*2 );
		float invs = 1 / s;

		return glm::quat(
			s * 0.5f, 
			rotationAxis.x * invs,
			rotationAxis.y * invs,
			rotationAxis.z * invs
		);

	}

	void updateOrientation(glm::vec3 newPos[3], glm::vec3 startPos[3])
    {
		glm::vec3 s1 = startPos[0] - startPos[1];
		glm::vec3 s2 = startPos[0] - startPos[2];
		glm::vec3 sn = glm::normalize(glm::cross(s1, s2));

		glm::vec3 t1 = newPos[0] - newPos[1];
		glm::vec3 t2 = newPos[0] - newPos[2];
		glm::vec3 tn = glm::normalize(glm::cross(t1, t2));


		//glm::quat q1(sn, tn);
		//glm::quat q2(q1*(s1-s2), t1-t2);
		//m_qatOrient = q2;
		//rotX = glm::acos(n[2]);
		//rotY = glm::atan(n[1]/n[0]);
		//rotZ = 0;


		/*
		Source=(s1,s2,s3)

		Target=(t1,t2,t3)

		NormSource = (s1 - s2)cross(s1 - s3)

		NormTarget = (t1 - t2)cross(t1 - t3)

		Quat1 = getRotationTo (NormSource,NormTarget)

		Quat2 = getRotationTo ( Quat1 * (s1 - s2),(t2 - t1) );

		QuatFinal = Quat2 * Quat1
		*/
		glm::quat qat1 = RotationBetweenVectors(sn,tn);
		glm::quat qat2 = RotationBetweenVectors(qat1 * (s1 - s2),(t2 - t1));
		m_qatOrient = qat2 * qat1;
    }
    // Update is called once per frame
    void Update()
    {

    }
	void test()
	{
		glm::vec4 vecDist(4, 4, 4, 0x00);
		glm::vec4 vecAngl(0.6, 0.6, 0.6, 0x00);
		glm::vec4 vecX = NewtonsNewMethod(vecDist,vecAngl);
		
	}
};

CTracking *g_pTracking = NULL;
#if defined(WIN32)
extern "C" {
	VRTRACKPLUGIN_API int startTrack()
	{
		CTracking *tracking = new CTracking();
	#ifdef WIN32
		int iResult;
		WSADATA wsaData;
		iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
		if (iResult != 0) {
			printf("WSAStartup failed: %d\n", iResult);
			return -1;
		}
	#endif
		int res = tracking->Start();
		g_pTracking = tracking;
		return res;
	}
	VRTRACKPLUGIN_API void stopTrack()
	{
		if(g_pTracking) {
			g_pTracking->m_fRun = 0;
#ifdef WIN32
			closesocket(g_pTracking->sockfd);
			WaitForSingleObject(g_pTracking->hThread,1000);
#else // ANDROID
			close(g_pTracking->sockfd);
			pthread_join(&g_pTracking->hThread);
#endif
		}
	}
	VRTRACKPLUGIN_API void getPosition(float pos[])
	{
		if(g_pTracking) {
			pos[0]=g_pTracking->newPos[0][0];
			pos[1]=g_pTracking->newPos[0][1];
			pos[2]=g_pTracking->newPos[0][2];

			pos[3]=g_pTracking->m_qatOrient.x;
			pos[4]=g_pTracking->m_qatOrient.y;
			pos[5]=g_pTracking->m_qatOrient.z;

			pos[6]=g_pTracking->m_qatOrient.w;
		}
	}
}

#elif defined(ANDROID)

#endif
#ifdef WIN32
int _tmain(int argc, _TCHAR* argv[])
#else
int main(int argc, char* argv[])
#endif
{
	CTracking *tracking = new CTracking();
#ifdef WIN32
	int iResult;
	WSADATA wsaData;
	iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed: %d\n", iResult);
		return 1;
	}
#endif
	tracking->Start();
	//tracking->test();
#ifdef WIN32
	while(1) Sleep(1000);
#endif
	return 0;
}


