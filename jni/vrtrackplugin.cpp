//#include <jni.h>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/trigonometric.hpp>
#include <glm/exponential.hpp>
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

#define MAX_UDP_RCV_LEN 1500
class CTracking
{
public:
#ifdef WIN32
	DWORD   dwThreadId;
	HANDLE  hThread;
#else
	pthread_t thread;
#endif
	glm::vec3 newPos;
	glm::vec3 startPos;
	int lineNumber;

	int sockfd;
	int remote_portno;
	int n;
	struct sockaddr_in serveraddr;
	struct sockaddr_in myaddr;
	int local_portno;

	int serverlen;
	char buf[MAX_UDP_RCV_LEN];

	float m_RawB1, m_RawB2, m_RawB3;
	float m_Rawtheta1, m_Rawtheta2, m_Rawtheta3;

	float m_B1, m_B2, m_B3;
	float m_theta1, m_theta2, m_theta3;
	float m_cos12, m_cos13, m_cos23;

	// TODO: Fill the values
	float m_RA, m_RB, m_RC;

	float period;
	bool anglesReady;
	bool altitudeReady;
	bool azthmusReady;
	bool firstPosition;
	float A_offsetX, A_offsetY, A_offsetZ;

    CTracking()
	{
		newPos = glm::vec3(0);
		startPos = glm::vec3(0);

		lineNumber = 1;

		m_RA = 1, m_RB = 1, m_RC = 1;

		A_offsetX = 0.0;
		A_offsetY = 0.0;
		A_offsetZ = 0.0;

		period = 1111122;
	
		firstPosition = false;
		remote_portno = 0;
		local_portno = 59427;
		memset(&serveraddr, 0x00, sizeof(serveraddr));
		memset(&myaddr, 0x00, sizeof(myaddr));
		clearPosState();
	}
	void clearPosState()
	{
		anglesReady = false;
		altitudeReady = false;
		azthmusReady = false;

		m_RawB1 = m_RawB2 = m_RawB3 = 0;
		m_Rawtheta1 = m_Rawtheta2 = m_Rawtheta3 = 0;

		m_B1 = m_B2 = m_B3 = 0;
		m_theta1 = m_theta2 = m_theta3 = 0;

	}
	// Use this for initialization
	void Start() {
		sockfd = socket(AF_INET, SOCK_DGRAM, 0);
		if (sockfd < 0) {
			//error("ERROR opening socket");
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
			return;
		}
		//startPos = this.transform.position; //test();
		printf("Client Started");
#ifdef WIN32
	hThread = CreateThread( NULL, 0, ReceiveData, this, 0, &dwThreadId);
#else
		if (pthread_create(&thread, NULL, ReceiveData, NULL)) {

		}
#endif
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
		while (true) {
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
			pTracking->parseSensorData(data);
			glm::vec4 vecAngl = pTracking->extractAngles();
			if (pTracking->altitudeReady && pTracking->azthmusReady) {
				glm::vec4 vecDist(2.5, 3, 4, 0x00);
				glm::vec4 vecX = pTracking->NewtonsNewMethod(vecDist, vecAngl);
				glm::vec4 vecBeta(pTracking->m_B1, pTracking->m_B2, pTracking->m_B3, 0x00);
				glm::vec4 vecTheta(pTracking->m_theta1, pTracking->m_theta2, pTracking->m_theta3, 0x00);
				pTracking->updatePosition(vecX, vecBeta, vecTheta);
			}
		}
		return 0;
	}

    void parseSensorData(char *data)
    {
        if (data[1] == 'x')
            parseSensorDataX(data);
        else if (data[1] == 'y')
            parseSensorDataY(data);
    }
#define NUM_WIDTH 9
#define NUM_POS_1 3
#define NUM_POS_2 13
#define NUM_POS_3 23

    void parseSensorDataX(char *data)
    {
        float tempB1, tempB2, tempB3;
        char szFloatNum[16]={0};
        
		memcpy(szFloatNum, data+NUM_POS_1, NUM_WIDTH);
        m_RawB1 = atof(szFloatNum);
		tempB1 = m_RawB1 / period * 360; // (float.Parse(splitArray[1]) / period) * 360;
       
		memcpy(szFloatNum, data+NUM_POS_2, NUM_WIDTH);
		m_RawB2 = atof(szFloatNum);
        tempB2 =  m_RawB2 / period * 360;
        
		memcpy(szFloatNum, data+NUM_POS_3, NUM_WIDTH);
		m_RawB3 = atof(szFloatNum);
        tempB3 = m_RawB3 / period * 360;
		/*
        if (tempB1 > 180 || tempB2 > 180 || tempB3 > 180) {
            //printf("ERROR: TRACKING LOST\n");
            //print(splitArray[0] + splitArray[1] + "" + splitArray[2] + splitArray[3]);
            return;
        } 
		else 
		*/
		{
            altitudeReady = true;
            //print("M_B1: " + tempB1 + " M_B2: " + tempB2 + "M_B3: " + tempB3);
            m_B1 = glm::radians(tempB1);//tempB1 * Mathf.Deg2Rad;
            m_B2 = glm::radians(tempB2);//tempB2 * Mathf.Deg2Rad;
            m_B3 = glm::radians(tempB3);;//tempB3 * Mathf.Deg2Rad;
        }
    }

    void parseSensorDataY(char *data)
    {
        float tempTheta1, tempTheta2, tempTheta3;
        char szFloatNum[10]={0};
        memcpy(szFloatNum, data+NUM_POS_1, NUM_WIDTH);
		m_Rawtheta1 = atof(szFloatNum);
        tempTheta1 = m_Rawtheta1 / period * 360;
        memcpy(szFloatNum, data+NUM_POS_2, NUM_WIDTH);
		m_Rawtheta2 = atof(szFloatNum);
        tempTheta2 = m_Rawtheta2 / period * 360;
        memcpy(szFloatNum, data+NUM_POS_3, NUM_WIDTH);
		m_Rawtheta3 = atof(szFloatNum);
        tempTheta3 = m_Rawtheta3 / period * 360;;
		/*
        if (tempTheta1 > 180 || tempTheta2 > 180 || tempTheta3 > 180) {
            //print("ERROR: TRACKING LOST"); print(splitArray[0] + splitArray[1] + "" + splitArray[2] + splitArray[3]);
            return;
        } 
		else 
		*/
		{
            azthmusReady = true;
            //print("M_Theta1: " + tempTheta1 + " M_Theta2: " + tempTheta2 + "M_Theta3: " + tempTheta3);
            m_theta1 = glm::radians(tempTheta1);// * Mathf.Deg2Rad;
            m_theta2 = glm::radians(tempTheta2);// * Mathf.Deg2Rad;
            m_theta3 = glm::radians(tempTheta3);// * Mathf.Deg2Rad;
        }
    }

    glm::vec4 extractAngles()
    {
		glm::vec4 vecAngl;
        vecAngl[0] = (glm::sin(m_B1) * glm::cos(m_theta1) * glm::sin(m_B2) * glm::cos(m_theta2) + (glm::sin(m_B1) * glm::sin(m_theta1) * glm::sin(m_B2) * glm::sin(m_theta2) + glm::cos(m_B1) * glm::cos(m_B2)));
        vecAngl[1] = (glm::sin(m_B2) * glm::cos(m_theta2) * glm::sin(m_B3) * glm::cos(m_theta3) + (glm::sin(m_B2) * glm::sin(m_theta2) * glm::sin(m_B3) * glm::sin(m_theta3) + glm::cos(m_B2) * glm::cos(m_B3)));
        vecAngl[2] = (glm::sin(m_B1) * glm::cos(m_theta1) * glm::sin(m_B3) * glm::cos(m_theta3) + (glm::sin(m_B1) * glm::sin(m_theta1) * glm::sin(m_B3) * glm::sin(m_theta3) + glm::cos(m_B1) * glm::cos(m_B3)));
        anglesReady = true;
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

			if(0)
			{
				float p1_RA, p1_RB, p1_RC; float p2_RA, p2_RB, p2_RC; 
				float p3_RA, p3_RB, p3_RC; 
				p1_RA = ((2 * ra) - (2 * rb * m_cos12)); 
				p1_RB = ((2 * rb) - (2 * ra * m_cos12)); 
				p1_RC = 0; 
				p2_RA = 0; 
				p2_RB = ((2 * rb) - (2 * rc * m_cos23)); 
				p2_RC = ((2 * rc) - (2 * rb * m_cos23)); 
				p3_RA = ((2 * ra) - (2 * rc * m_cos13)); 
				p3_RB = 0; 
				p3_RC = ((2 * rc) - (2 * ra * m_cos13));
				float sm1_const, sm2_const, sm3_const; 
				float sm1_RA, sm1_RB, sm1_RC; sm1_RA = p1_RA; 
				sm1_RB = p1_RB; 
				sm1_RC = p1_RC; 
				sm1_const = (-ra * p1_RA) + (-rb * p1_RB) + (-rc * p1_RC);
				float sm2_RA, sm2_RB, sm2_RC; 
				sm2_RA = p2_RA; sm2_RB = p2_RB; 
				sm2_RC = p2_RC; sm2_const = (-ra * p2_RA) + (-rb * p2_RB) + (-rc * p2_RC);
				float sm3_RA, sm3_RB, sm3_RC; 
				sm3_RA = p3_RA; sm3_RB = p3_RB; 
				sm3_RC = p3_RC; 
				sm3_const = (-ra * p3_RA) + (-rb * p3_RB) + (-rc * p3_RC);
				vecB[0] -= sm1_const;
				vecB[1] -= sm2_const;
				vecB[2] -= sm3_const;
			}




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
/*
		m_RA = vecTmp[0]; 
		m_RB = vecTmp[1]; 
		m_RC = vecTmp[2];
        if (firstPosition == false) {
            firstPosition = true;
            A_offsetX = glm::abs(m_RA) * glm::sin(m_B1) * glm::cos(m_theta1) - startPos.x;
            A_offsetY = glm::abs(m_RA) * glm::sin(m_B1) * glm::sin(m_theta1) - startPos.y;
            A_offsetZ = glm::abs(m_RA) * glm::sin(m_B1) - startPos.z;
            //print("OFFSETX: " + A_offsetX + " OFFSETY: " + A_offsetY + "OFFSETZ: " + A_offsetZ);
        }
		*/
    }

 
    void updatePosition(glm::vec4 vecX, glm::vec4 vecBeta, glm::vec4 vecTheta)
    {
		static int count = 0;
		float ra = vecX[0]; 
		float rb = vecX[1]; 
		float rc = vecX[2];
		float b1 = vecBeta[0];
		float t1 = vecTheta[0];

        float x = glm::abs(ra) * glm::sin(b1) * glm::cos(t1);
        float y = glm::abs(ra) * glm::sin(b1) * glm::sin(t1);
        float z = glm::abs(ra) * glm::cos(b1);
        newPos = glm::vec3(-(x - A_offsetX), -(z - A_offsetZ), -(y - A_offsetY));
		if(count++ % 60 == 0) {
			printf("Pos:x=%3.6f y=%3.6f z=%3.6f RA=%3.6f RB=%3.6f RC=%3.6f m_B1=%3.6f m_theta1=%3.6f\n", x,y,z,ra,rb,rc, b1, t1);
		}
		clearPosState();
    }

    // Update is called once per frame
    void Update()
    {

    }
};

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
	if(0)
	{
		glm::vec4 vecDist(4, 4, 4, 0x00);
		glm::vec4 vecAngl(0.6, 0.6, 0.6, 0x00);
		tracking->NewtonsNewMethod(vecDist,vecAngl);
	}
#ifdef WIN32
	while(1) Sleep(1000);
#endif
	return 0;
}


