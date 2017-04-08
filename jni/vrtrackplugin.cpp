#include <jni.h>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/trigonometric.hpp>
#include <glm/exponential.hpp>
#include <math.h>
#include <string>

#ifdef WIN32
#else // LINUX
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#endif

#define MAX_UDP_RCV_LEN 1500
class CTracking
{
	pthread_t thread;
	glm::vec3 newPos;
	glm::vec3 startPos;
	int lineNumber;

	int sockfd, portno, n;
	struct sockaddr_in serveraddr;
	int serverlen;
	char buf[MAX_UDP_RCV_LEN];

	float m_B1, m_B2, m_B3;
	float m_theta1, m_theta2, m_theta3;
	float m_cos12, m_cos13, m_cos23;

	// TODO: Fill the values
	float m_distAB, m_distAC, m_distBC;
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
		m_B1 = 0,
		m_B2 = 0,
		m_B3 = 0;
		m_theta1 = 0,
		m_theta2 = 0,
		m_theta3 = 0;

		m_RA = 1, m_RB = 1, m_RC = 1;

		period = 1111122;
		anglesReady = false;
		altitudeReady = false;
		azthmusReady = false;
		firstPosition = false;
	}
	// Use this for initialization
	void Start() {
		sockfd = socket(AF_INET, SOCK_DGRAM, 0);
		if (sockfd < 0) {
			//error("ERROR opening socket");
		}
		serveraddr.sin_family = AF_INET;
		serveraddr.sin_port = htons(portno);
		serverlen = sizeof(serveraddr);

		//startPos = this.transform.position; //test();
		printf("Client Started");
		if (pthread_create(&thread, NULL, ReceiveData, NULL)) {

		}
	}

    int getData(char *data, int nLen)
    {
    	int bytesRecvd = recvfrom(sockfd, buf, nLen, 0, (const sockaddr*)&serveraddr, &serverlen);
    	return bytesRecvd;
    }
	static void* ReceiveData(void *pArg) {
		CTracking *pTracking = (CTracking*) pArg;
		char data[256];
		while (true) {
			printf("client.Receive");
			pTracking->n = pTracking->getData(data, 256);
			if (pTracking->n <= 0)
				continue;
			pTracking->getSensorData(data);
			pTracking->extractAngles();
			if (pTracking->altitudeReady && pTracking->azthmusReady) {
				pTracking->NewtonsNewMethod();
				pTracking->updatePosition();
			}
		}
	}

    void getSensorData(char *data)
    {
        if (data[0] == 'x')
            getSensorDataX(data);
        else if (data[0] == 'y')
            getSensorDataY(data);
    }
#define NUM_WIDTH 10
#define NUM_POS_1 2
#define NUM_POS_2 12
#define NUM_POS_3 22

    void getSensorDataX(char *data)
    {
        float tempB1, tempB2, tempB3;
        char szFloatNum[10]={0};
        memcpy(szFloatNum, data+NUM_POS_1, NUM_WIDTH);
        tempB1 = atof(szFloatNum) / period * 360; // (float.Parse(splitArray[1]) / period) * 360;
        memcpy(szFloatNum, data+NUM_POS_2, NUM_WIDTH);
        tempB2 = atof(szFloatNum) / period * 360;
        memcpy(szFloatNum, data+NUM_POS_3, NUM_WIDTH);
        tempB3 = atof(szFloatNum) / period * 360;

        if (tempB1 > 180 || tempB2 > 180 || tempB3 > 180) {
            printf("ERROR: TRACKING LOST");
            //print(splitArray[0] + splitArray[1] + "" + splitArray[2] + splitArray[3]);
            return;
        } else {
            altitudeReady = true;
            //print("M_B1: " + tempB1 + " M_B2: " + tempB2 + "M_B3: " + tempB3);
            m_B1 = glm::radians(tempB1);//tempB1 * Mathf.Deg2Rad;
            m_B2 = glm::radians(tempB2);//tempB2 * Mathf.Deg2Rad;
            m_B3 = glm::radians(tempB3);;//tempB3 * Mathf.Deg2Rad;
        }
    }

    void getSensorDataY(char *data)
    {
        float tempTheta1, tempTheta2, tempTheta3;
        char szFloatNum[10]={0};
        memcpy(szFloatNum, data+NUM_POS_1, NUM_WIDTH);
        tempTheta1 = atof(szFloatNum) / period * 360;
        memcpy(szFloatNum, data+NUM_POS_1, NUM_WIDTH);
        tempTheta2 = atof(szFloatNum) / period * 360;
        memcpy(szFloatNum, data+NUM_POS_1, NUM_WIDTH);
        tempTheta3 = atof(szFloatNum) / period * 360;;
        if (tempTheta1 > 180 || tempTheta2 > 180 || tempTheta3 > 180) {
            //print("ERROR: TRACKING LOST"); print(splitArray[0] + splitArray[1] + "" + splitArray[2] + splitArray[3]);
            return;
        } else {
            azthmusReady = true;
            //print("M_Theta1: " + tempTheta1 + " M_Theta2: " + tempTheta2 + "M_Theta3: " + tempTheta3);
            m_theta1 = glm::radians(tempTheta1);// * Mathf.Deg2Rad;
            m_theta2 = glm::radians(tempTheta2);// * Mathf.Deg2Rad;
            m_theta3 = glm::radians(tempTheta3);// * Mathf.Deg2Rad;
        }
    }

    void extractAngles()
    {
        m_cos12 = (glm::sin(m_B1) * glm::cos(m_theta1) * glm::sin(m_B2) * glm::cos(m_theta2) + (glm::sin(m_B1) * glm::sin(m_theta1) * glm::sin(m_B2) * glm::sin(m_theta2) + glm::cos(m_B1) * glm::cos(m_B2)));
        m_cos23 = (glm::sin(m_B2) * glm::cos(m_theta2) * glm::sin(m_B3) * glm::cos(m_theta3) + (glm::sin(m_B2) * glm::sin(m_theta2) * glm::sin(m_B3) * glm::sin(m_theta3) + glm::cos(m_B2) * glm::cos(m_B3)));
        m_cos13 = (glm::sin(m_B1) * glm::cos(m_theta1) * glm::sin(m_B3) * glm::cos(m_theta3) + (glm::sin(m_B1) * glm::sin(m_theta1) * glm::sin(m_B3) * glm::sin(m_theta3) + glm::cos(m_B1) * glm::cos(m_B3)));
        anglesReady = true;
    }
     void NewtonsNewMethod() {
        int iterations = 0;
        float tempRA = 1, tempRB = 1, tempRC = 1;
        float gRA, gRB, gRC;
        for (int i = 0; i < 50; i++) {
            gRA = -(glm::pow(tempRA, 2) + glm::pow(tempRB, 2) - (2 * tempRA * tempRB * m_cos12) - glm::pow(m_distAB, 2));
            gRB = -(glm::pow(tempRB, 2) + glm::pow(tempRC, 2) - (2 * tempRB * tempRC * m_cos23) - glm::pow(m_distBC, 2));
            gRC = -(glm::pow(tempRA, 2) + glm::pow(tempRC, 2) - (2 * tempRA * tempRC * m_cos13) - glm::pow(m_distAC, 2));
            float p1_RA, p1_RB, p1_RC; float p2_RA, p2_RB, p2_RC;
            float p3_RA, p3_RB, p3_RC;
            p1_RA = ((2 * tempRA) - (2 * tempRB * m_cos12));
            p1_RB = ((2 * tempRB) - (2 * tempRA * m_cos12));
            p1_RC = 0; p2_RA = 0;
            p2_RB = ((2 * tempRB) - (2 * tempRC * m_cos23));
            p2_RC = ((2 * tempRC) - (2 * tempRB * m_cos23));
            p3_RA = ((2 * tempRA) - (2 * tempRC * m_cos13));
            p3_RB = 0; p3_RC = ((2 * tempRC) - (2 * tempRA * m_cos13));
            float sm1_const, sm2_const, sm3_const;
            float sm1_RA, sm1_RB, sm1_RC; sm1_RA = p1_RA;
            sm1_RB = p1_RB; sm1_RC = p1_RC;
            sm1_const = (-tempRA * p1_RA) + (-tempRB * p1_RB) + (-tempRC * p1_RC);
            float sm2_RA, sm2_RB, sm2_RC;
            sm2_RA = p2_RA; sm2_RB = p2_RB;
            sm2_RC = p2_RC; sm2_const = (-tempRA * p2_RA) + (-tempRB * p2_RB) + (-tempRC * p2_RC);
            float sm3_RA, sm3_RB, sm3_RC;
            sm3_RA = p3_RA; sm3_RB = p3_RB;
            sm3_RC = p3_RC; sm3_const = (-tempRA * p3_RA) + (-tempRB * p3_RB) + (-tempRC * p3_RC);

            glm::mat4 matrixA;// = Matrix4x4.identity;
            matrixA[0][0] = sm1_RA;
            matrixA[0][1] = sm1_RB;
            matrixA[0][2] = sm1_RC;
            matrixA[1][0] = sm2_RA;
            matrixA[1][1] = sm2_RB;
            matrixA[1][2] = sm2_RC;

            matrixA[2][0] = sm3_RA;
            matrixA[2][1] = sm3_RB;
            matrixA[2][2] = sm3_RC;
            glm::mat4 matrixB(0.0f);// = glm::mat4.zero;

            matrixB[0][3] = (gRA - sm1_const); matrixB[1][3] = (gRB - sm2_const); matrixB[2][3] = (gRC - sm3_const);
            glm::mat4 solution = glm::inverse(matrixA) * matrixB;

            if (tempRA == solution[0][3] && tempRB == solution[1][3] && tempRC == solution[2][3]) {
                printf("Problem Solved!"); break;
            } else {
                tempRA = solution[0][3];
                tempRB = solution[1][3];
                tempRC = solution[2][3];
            }
            iterations++;

            if (i == 29) {
                if (gRA > .001f || gRB > .001f || gRB > .001f) i--;
            }
        }

        gRA = (glm::pow(tempRA, 2) + glm::pow(tempRB, 2) - (2 * tempRA * tempRB * m_cos12) - glm::pow(m_distAB, 2));
        gRB = (glm::pow(tempRB, 2) + glm::pow(tempRC, 2) - (2 * tempRB * tempRC * m_cos23) - glm::pow(m_distBC, 2));
        gRC = (glm::pow(tempRA, 2) + glm::pow(tempRC, 2) - (2 * tempRA * tempRC * m_cos13) - glm::pow(m_distAC, 2));

        m_RA = tempRA; m_RB = tempRB; m_RC = tempRC;

        if (firstPosition == false) {
            firstPosition = true;
            A_offsetX = glm::abs(m_RA) * glm::sin(m_B1) * glm::cos(m_theta1) - startPos.x;
            A_offsetY = glm::abs(m_RA) * glm::sin(m_B1) * glm::sin(m_theta1) - startPos.y;
            A_offsetZ = glm::abs(m_RA) * glm::sin(m_B1) - startPos.z;
            //print("OFFSETX: " + A_offsetX + " OFFSETY: " + A_offsetY + "OFFSETZ: " + A_offsetZ);
        }
    }

     void NewtonsTestMethod()
    {
		int i;
        int iterations = 0; //Initial Point to start with
        float tempRA = 1, tempRB = 1, tempRC = 1;
        float gRA, gRB, gRC;
        for (i = 0; i < 30; i++)
        {
            // TODO: Make sure these are correct (Double Check)
            gRA = -(glm::pow(tempRA, 2) + glm::pow(tempRB, 2) - (2 * tempRA * tempRB * m_cos12) - glm::pow(m_distAB, 2));
            gRB = -(glm::pow(tempRB, 2) + glm::pow(tempRC, 2) - (2 * tempRB * tempRC * m_cos23) - glm::pow(m_distBC, 2));
            gRC = -(glm::pow(tempRA, 2) + glm::pow(tempRC, 2) - (2 * tempRA * tempRC * m_cos13) - glm::pow(m_distAC, 2));
            //print("gRA: " + gRA); print("gRB: " + gRB);
            //print("gRC: " + gRC);
            float p1_RA, p1_RB, p1_RC;
            float p2_RA, p2_RB, p2_RC;
            float p3_RA, p3_RB, p3_RC;
            p1_RA = ((2 * tempRA) - (2 * tempRB * m_cos12));
            p1_RB = ((2 * tempRB) - (2 * tempRA * m_cos12));
            p1_RC = 0;
            //print(lineNumber + " Jacobian1: " + p1_RA + " + " + p1_RB + " + " + p1_RC);
            lineNumber++;
            p2_RA = 0; p2_RB = ((2 * tempRB) - (2 * tempRC * m_cos23)); p2_RC = ((2 * tempRC) - (2 * tempRB * m_cos23));
            //print(lineNumber + " Jacobian2: " + p2_RA + " + " + p2_RB + " + " + p2_RC); lineNumber++; p3_RA = ((2 * tempRA) - (2 * tempRC * m_cos13));
            p3_RB = 0; p3_RC = ((2 * tempRC) - (2 * tempRA * m_cos13));
            //print(lineNumber + " Jacobian3: " + p3_RA + " + " + p3_RB + " + " + p3_RC);
            lineNumber++;
            // Now it's time to create the submatrix and solve it with the jacobian method.
            float sm1_const, sm2_const, sm3_const;
            float sm1_RA, sm1_RB, sm1_RC;
            sm1_RA = p1_RA;
            sm1_RB = p1_RB;
            sm1_RC = p1_RC;
            sm1_const = (-tempRA * p1_RA) + (-tempRB * p1_RB) + (-tempRC * p1_RC); // sm1_RA + sm1_RB + sm1_RC + sm1_const = gRA
            float sm2_RA, sm2_RB, sm2_RC;
            sm2_RA = p2_RA;
            sm2_RB = p2_RB;
            sm2_RC = p2_RC; sm2_const = (-tempRA * p2_RA) + (-tempRB * p2_RB) + (-tempRC * p2_RC);
            float sm3_RA, sm3_RB, sm3_RC; sm3_RA = p3_RA; sm3_RB = p3_RB; sm3_RC = p3_RC;
            sm3_const = (-tempRA * p3_RA) + (-tempRB * p3_RB) + (-tempRC * p3_RC);
            glm::mat4 matrixA;
            matrixA[0][0] = sm1_RA;
            matrixA[0][1] = sm1_RB;
            matrixA[0][2] = sm1_RC;
            matrixA[1][0] = sm2_RA;
            matrixA[1][1] = sm2_RB;
            matrixA[1][2] = sm2_RC;
            matrixA[2][0] = sm3_RA;
            matrixA[2][1] = sm3_RB;
            matrixA[2][2] = sm3_RC;
            glm::mat4 matrixB(0.0f);

            matrixB[0][3] = (gRA - sm1_const);
            matrixB[1][3] = (gRB - sm2_const);
            matrixB[2][3] = (gRC - sm3_const);
            //print("SM1_Const: " + sm1_const);
            //print("SM2_Const: " + sm2_const);
            //print("SM3_Const: " + sm3_const);
            //print(lineNumber + " Equals: " + (gRA - sm1_const) + ", " + (gRB - sm2_const) + ", " + (gRC - sm3_const));
            lineNumber++;
            glm::mat4 solution = glm::inverse(matrixA) * matrixB;
            if (tempRA == solution[0][3] && tempRB == solution[1][3] && tempRC == solution[2][3])
            {
                printf("Problem Solved!");
                break;
            }
            else
            {
                tempRA = solution[0][3];
                tempRB = solution[1][3];
                tempRC = solution[2][3];
                //print(lineNumber + "Solution: " + tempRA + ", " + tempRB + ", " + tempRC);
                lineNumber++;
            }
            iterations++;

            if (i == 29)
            {
                if (gRA > .001f || gRB > .001f || gRB > .001f) i--;
            }
        }
        gRA = (glm::pow(tempRA, 2) + glm::pow(tempRB, 2) - (2 * tempRA * tempRB * m_cos12) - glm::pow(m_distAB, 2));
        gRB = (glm::pow(tempRB, 2) + glm::pow(tempRC, 2) - (2 * tempRB * tempRC * m_cos23) - glm::pow(m_distBC, 2));
        gRC = (glm::pow(tempRA, 2) + glm::pow(tempRC, 2) - (2 * tempRA * tempRC * m_cos13) - glm::pow(m_distAC, 2));

        //print(lineNumber + " RA got this close to zero: " + gRA);
        lineNumber++;
        //print(lineNumber + " RB got this close to zero: " + gRB);
        lineNumber++;
        //print(lineNumber + " RC got this close to zero: " + gRC);
        lineNumber++;

        m_RA = tempRA; m_RB = tempRB; m_RC = tempRC;
        //print(lineNumber + " RA: " + m_RA + " At " + iterations);
        lineNumber++;
        anglesReady = false;
    }

    void updatePosition()
    {
        float x = glm::abs(m_RA) * glm::sin(m_B1) * glm::cos(m_theta1);
        float y = glm::abs(m_RA) * glm::sin(m_B1) * glm::sin(m_theta1);
        float z = glm::abs(m_RA) * glm::cos(m_B1);
        newPos = glm::vec3(-(x - A_offsetX), -(z - A_offsetZ), -(y - A_offsetY));
    }

    // Update is called once per frame
    void Update()
    {

    }

	void test() {
		char data[256];
		int len = 0;
		len = getData(data, 256);

		if (len < 0)
			return;
		getSensorData(data);
		len = getData(data, 256);
		if (len < 0)
			return;
		getSensorData(data);
		extractAngles();
		NewtonsTestMethod();
	}
};

#ifdef WIN32
int _tmain(int argc, _TCHAR* argv[])
#else
int main(int argc, char* argv[])
#endif
{
	return 0;
}


