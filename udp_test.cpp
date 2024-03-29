// udp_test.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <WinSock2.h>
#include <string>
typedef struct Vector3
{
	int x,y,z;
};

typedef int Matrix4x4[4][4];
class track
{

    Thread receiveThread;
    Vector3 newPos;
    Vector3 startPos;
    private IPEndPoint anyIP;
    UdpClient client;
    bool firstTime;
    int globalCameraRefresh;
    int lineNumber;

    std::string lastReceivedUDPPacket;
    float m_B1,
        m_B2,
        m_B3;
    float m_theta1,
        m_theta2,
        m_theta3;
    float m_cos12, m_cos13, m_cos23;
     float m_distAB, m_distAC, m_distBC;
    float m_RA, m_RB, m_RC;
    float syncFlashTime; // SyncFlash time in cycles for a 16mhz clock

    float period;
     bool anglesReady;
    bool altitudeReady;
    bool azthmusReady;
    bool firstPosition;
    int numSweeps;
     std::string IP;
    float A_offsetX, A_offsetY, A_offsetZ;

	track()
	{
		newPos = {0};
		startPos = {0};
		int port = 8080; // define > init 
		IPEndPoint anyIP;
		UdpClient client;
		firstTime = false;
		globalCameraRefresh = 0;
		lineNumber = 1;

		lastReceivedUDPPacket = "";
		m_B1 = 0,
		m_B2 = 0,
		m_B3 = 0;
		m_theta1 = 0,
		m_theta2 = 0,
		m_theta3 = 0;
		
		m_RA = 1, m_RB = 1, m_RC = 1;
		syncFlashTime = 133280; // SyncFlash time in cycles for a 16mhz clock

		period = 1111122;
		anglesReady = false;
		altitudeReady = false;
		azthmusReady = false;
		firstPosition = false;
		numSweeps = 0;
	}
    // Use this for initialization
    void Start()
    {
        anyIP = new IPEndPoint(IPAddress.Any, 0unity );
        client = new UdpClient(59427);
        startPos = this.transform.position; //test(); 
        print("Client Started");
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
    }

    void ReceiveData()
    {
        while (true)
        {
            try
            {
                print("client.Receive");
                byte[] data = client.Receive(ref anyIP);
                string text = ""; text = Encoding.ASCII.GetString(data);
                string[] splitArray = parseData(text);
                if (splitArray == null) 
                    continue;
                lastReceivedUDPPacket = text; 
                //print("UNVALIDATED: " + splitArray[0] + splitArray[1] + "" + splitArray[2] + splitArray[3]); 
                inputValidation(splitArray);
                extractAngles();
                if (altitudeReady == true && azthmusReady == true)
                {
                    NewtonsNewMethod(); 
                    updatePosition();
                }
            }
            catch (Exception err)
            {
                print(err.ToString());
            }
        }
    }

    void inputValidation(string[] splitArray)
    {
        if (splitArray[0] == "x")
            validateX(splitArray);
        else if (splitArray[0] == "y")
            validateY(splitArray);
    }

    void validateX(string[] splitArray)
    {
        float tempB1, tempB2, tempB3; tempB1 = (float.Parse(splitArray[1]) / period) * 360;
        tempB2 = (float.Parse(splitArray[2]) / period) * 360; tempB3 = (float.Parse(splitArray[3]) / period) * 360;
        if (tempB1 > 180 || tempB2 > 180 || tempB3 > 180)
        {
            print("ERROR: TRACKING LOST");
            print(splitArray[0] + splitArray[1] + "" + splitArray[2] + splitArray[3]);
            return;
        }
        else
        {
            altitudeReady = true;
            print("M_B1: " + tempB1 + " M_B2: " + tempB2 + "M_B3: " + tempB3); m_B1 = tempB1 * Mathf.Deg2Rad; m_B2 = tempB2 * Mathf.Deg2Rad; m_B3 = tempB3 * Mathf.Deg2Rad;
        }
    }
    void validateY(string[] splitArray)
    {
        float tempTheta1, tempTheta2, tempTheta3;
        tempTheta1 = (float.Parse(splitArray[1]) / period) * 360; tempTheta2 = (float.Parse(splitArray[2]) / period) * 360;
        tempTheta3 = (float.Parse(splitArray[3]) / period) * 360;
        if (tempTheta1 > 180 || tempTheta2 > 180 || tempTheta3 > 180)
        {
            print("ERROR: TRACKING LOST"); print(splitArray[0] + splitArray[1] + "" + splitArray[2] + splitArray[3]);
            return;
        }
        else
        {
            azthmusReady = true;
            print("M_Theta1: " + tempTheta1 + " M_Theta2: " + tempTheta2 + "M_Theta3: " + tempTheta3); m_theta1 = tempTheta1 * Mathf.Deg2Rad; m_theta2 = tempTheta2 * Mathf.Deg2Rad;
            m_theta3 = tempTheta3 * Mathf.Deg2Rad;
        }
    }

    string[] parseData(string text)
    {
        string[] array = new string[4];
        for (int c = 0; c < 4; c++) array[c] = "";
        if (text.Length == 31) text += '0';
        if (text.Length != 32)
        {
            print("ERROR: MESSAGE NOT 32 BYTES!");
            print("Length of Message: " + text.Length);
            for (int k = 0; k < text.Length; k++) print(k + "." + text[k].ToString());
            return null;
        }
        array[0] = text[1].ToString();
        for (int i = 2; i < 32; i++)
        {
            // Skip the commas in the data 
            if (i > 2 && i < 12) array[1] += text[i].ToString();
            if (i > 12 && i < 22) array[2] += text[i].ToString();
            if (i > 22 && i < 32) array[3] += text[i].ToString();
        }
        return array;
    }

    private void extractAngles()
    {
        m_cos12 = (Mathf.Sin(m_B1) * Mathf.Cos(m_theta1) * Mathf.Sin(m_B2) * Mathf.Cos(m_theta2) + (Mathf.Sin(m_B1) * Mathf.Sin(m_theta1) * Mathf.Sin(m_B2) * Mathf.Sin(m_theta2) + Mathf.Cos(m_B1) * Mathf.Cos(m_B2)));
        m_cos23 = (Mathf.Sin(m_B2) * Mathf.Cos(m_theta2) * Mathf.Sin(m_B3) * Mathf.Cos(m_theta3) + (Mathf.Sin(m_B2) * Mathf.Sin(m_theta2) * Mathf.Sin(m_B3) * Mathf.Sin(m_theta3) + Mathf.Cos(m_B2) * Mathf.Cos(m_B3)));
        m_cos13 = (Mathf.Sin(m_B1) * Mathf.Cos(m_theta1) * Mathf.Sin(m_B3) * Mathf.Cos(m_theta3) + (Mathf.Sin(m_B1) * Mathf.Sin(m_theta1) * Mathf.Sin(m_B3) * Mathf.Sin(m_theta3) + Mathf.Cos(m_B1) * Mathf.Cos(m_B3)));
        anglesReady = true;
    }
     void NewtonsNewMethod() { 
        int iterations = 0;
        float tempRA = 1, tempRB = 1, tempRC = 1;
        float gRA, gRB, gRC; 
        for (int i = 0; i < 50; i++) {
            gRA = -(Mathf.Pow(tempRA, 2) + Mathf.Pow(tempRB, 2) - (2 * tempRA * tempRB * m_cos12) - Mathf.Pow(m_distAB, 2)); 
            gRB = -(Mathf.Pow(tempRB, 2) + Mathf.Pow(tempRC, 2) - (2 * tempRB * tempRC * m_cos23) - Mathf.Pow(m_distBC, 2)); 
            gRC = -(Mathf.Pow(tempRA, 2) + Mathf.Pow(tempRC, 2) - (2 * tempRA * tempRC * m_cos13) - Mathf.Pow(m_distAC, 2));
            float p1_RA, p1_RB, p1_RC; float p2_RA, p2_RB, p2_RC; 
            float p3_RA, p3_RB, p3_RC; 
            p1_RA = ((2 * tempRA) - (2 * tempRB * m_cos12)); 
            p1_RB = ((2 * tempRB) - (2 * tempRA * m_cos12)); 
            p1_RC = 0; p2_RA = 0; 
            p2_RB = ((2 * tempRB) - (2 * tempRC * m_cos23)); 
            p2_RC = ((2 * tempRC) - (2 * tempRB * m_cos23)); 
            p3_RA = ((2 * tempRA) - (2 * tempRC * m_cos13)); 
            p3_RB = 0; 
			p3_RC = ((2 * tempRC) - (2 * tempRA * m_cos13));
            float sm1_const, sm2_const, sm3_const; 
            float sm1_RA, sm1_RB, sm1_RC; sm1_RA = p1_RA; 
            sm1_RB = p1_RB; 
			sm1_RC = p1_RC; 
            sm1_const = (-tempRA * p1_RA) + (-tempRB * p1_RB) + (-tempRC * p1_RC);
            float sm2_RA, sm2_RB, sm2_RC; 
            sm2_RA = p2_RA; sm2_RB = p2_RB; 
            sm2_RC = p2_RC; sm2_const = (-tempRA * p2_RA) + (-tempRB * p2_RB) + (-tempRC * p2_RC);
            float sm3_RA, sm3_RB, sm3_RC; 
            sm3_RA = p3_RA; sm3_RB = p3_RB; 
            sm3_RC = p3_RC; 
			sm3_const = (-tempRA * p3_RA) + (-tempRB * p3_RB) + (-tempRC * p3_RC);
            
            Matrix4x4 matrixA;// = Matrix4x4.identity; 
            matrixA[0][0] = sm1_RA; 
            matrixA[0, 1] = sm1_RB; 
            matrixA[0, 2] = sm1_RC;
            matrixA[1, 0] = sm2_RA; 
            matrixA[1, 1] = sm2_RB; 
            matrixA[1, 2] = sm2_RC;

            matrixA[2, 0] = sm3_RA;
            matrixA[2, 1] = sm3_RB;
            matrixA[2, 2] = sm3_RC;
            Matrix4x4 matrixB = Matrix4x4.zero;

            matrixB[0, 3] = (gRA - sm1_const); 
			matrixB[1, 3] = (gRB - sm2_const); 
			matrixB[2, 3] = (gRC - sm3_const);
            Matrix4x4 solution = matrixA.inverse * matrixB;

            if (tempRA == solution[0, 3] && tempRB == solution[1, 3] && tempRC == solution[2, 3]) { 
                print("Problem Solved!"); break; 
            } else { 
                tempRA = solution[0, 3]; 
                tempRB = solution[1, 3]; 
                tempRC = solution[2, 3]; 
            } 
            iterations++;

            if (i == 29) { 
                if (gRA > .001f || gRB > .001f || gRB > .001f) i--; 
            } 
        }

        gRA = (Mathf.Pow(tempRA, 2) + Mathf.Pow(tempRB, 2) - (2 * tempRA * tempRB * m_cos12) - Mathf.Pow(m_distAB, 2)); 
        gRB = (Mathf.Pow(tempRB, 2) + Mathf.Pow(tempRC, 2) - (2 * tempRB * tempRC * m_cos23) - Mathf.Pow(m_distBC, 2)); 
        gRC = (Mathf.Pow(tempRA, 2) + Mathf.Pow(tempRC, 2) - (2 * tempRA * tempRC * m_cos13) - Mathf.Pow(m_distAC, 2));

        m_RA = tempRA; m_RB = tempRB; m_RC = tempRC;

        if (firstPosition == false) { 
            firstPosition = true;
            A_offsetX = Mathf.Abs(m_RA) * Mathf.Sin(m_B1) * Mathf.Cos(m_theta1) - startPos.x; 
            A_offsetY = Mathf.Abs(m_RA) * Mathf.Sin(m_B1) * Mathf.Sin(m_theta1) - startPos.y; 
            A_offsetZ = Mathf.Abs(m_RA) * Mathf.Cos(m_B1) - startPos.z; 
            print("OFFSETX: " + A_offsetX + " OFFSETY: " + A_offsetY + "OFFSETZ: " + A_offsetZ); 
        } 
    }

     void OnApplicationQuit()
    {
        receiveThread.Abort();
        if (client != null)
        {
            client.Close();
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
            gRA = -(Mathf.Pow(tempRA, 2) + Mathf.Pow(tempRB, 2) - (2 * tempRA * tempRB * m_cos12) - Mathf.Pow(m_distAB, 2));
            gRB = -(Mathf.Pow(tempRB, 2) + Mathf.Pow(tempRC, 2) - (2 * tempRB * tempRC * m_cos23) - Mathf.Pow(m_distBC, 2));
            gRC = -(Mathf.Pow(tempRA, 2) + Mathf.Pow(tempRC, 2) - (2 * tempRA * tempRC * m_cos13) - Mathf.Pow(m_distAC, 2));
            print("gRA: " + gRA); print("gRB: " + gRB);
            print("gRC: " + gRC);
            float p1_RA, p1_RB, p1_RC;
            float p2_RA, p2_RB, p2_RC;
            float p3_RA, p3_RB, p3_RC; 
			p1_RA = ((2 * tempRA) - (2 * tempRB * m_cos12)); 
			p1_RB = ((2 * tempRB) - (2 * tempRA * m_cos12)); p1_RC = 0; print(lineNumber + " Jacobian1: " + p1_RA + " + " + p1_RB + " + " + p1_RC);
            lineNumber++;
            p2_RA = 0; p2_RB = ((2 * tempRB) - (2 * tempRC * m_cos23)); p2_RC = ((2 * tempRC) - (2 * tempRB * m_cos23));
            print(lineNumber + " Jacobian2: " + p2_RA + " + " + p2_RB + " + " + p2_RC); 
			lineNumber++; p3_RA = ((2 * tempRA) - (2 * tempRC * m_cos13));
            p3_RB = 0; p3_RC = ((2 * tempRC) - (2 * tempRA * m_cos13)); print(lineNumber + " Jacobian3: " + p3_RA + " + " + p3_RB + " + " + p3_RC);
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
            Matrix4x4 matrixA = Matrix4x4.identity;
            matrixA[0, 0] = sm1_RA;
            matrixA[0, 1] = sm1_RB;
            matrixA[0, 2] = sm1_RC;
            matrixA[1, 0] = sm2_RA;
            matrixA[1, 1] = sm2_RB;
            matrixA[1, 2] = sm2_RC;
            matrixA[2, 0] = sm3_RA;
            matrixA[2, 1] = sm3_RB;
            matrixA[2, 2] = sm3_RC;
            Matrix4x4 matrixB = Matrix4x4.zero;

            matrixB[0, 3] = (gRA - sm1_const);
            matrixB[1, 3] = (gRB - sm2_const);
            matrixB[2, 3] = (gRC - sm3_const);
            print("SM1_Const: " + sm1_const);
            print("SM2_Const: " + sm2_const);
            print("SM3_Const: " + sm3_const);
            print(lineNumber + " Equals: " + (gRA - sm1_const) + ", " + (gRB - sm2_const) + ", " + (gRC - sm3_const));
            lineNumber++;
            Matrix4x4 solution = matrixA.inverse * matrixB;
            if (tempRA == solution[0, 3] && tempRB == solution[1, 3] && tempRC == solution[2, 3])
            {
                print("Problem Solved!");
                break;
            }
            else
            {
                tempRA = solution[0, 3];
                tempRB = solution[1, 3];
                tempRC = solution[2, 3];
                print(lineNumber + "Solution: " + tempRA + ", " + tempRB + ", " + tempRC);
                lineNumber++;
            }
            iterations++;

            if (i == 29)
            {
                if (gRA > .001f || gRB > .001f || gRB > .001f) i--;
            }
        }
        gRA = (Mathf.Pow(tempRA, 2) + Mathf.Pow(tempRB, 2) - (2 * tempRA * tempRB * m_cos12) - Mathf.Pow(m_distAB, 2));
        gRB = (Mathf.Pow(tempRB, 2) + Mathf.Pow(tempRC, 2) - (2 * tempRB * tempRC * m_cos23) - Mathf.Pow(m_distBC, 2));
        gRC = (Mathf.Pow(tempRA, 2) + Mathf.Pow(tempRC, 2) - (2 * tempRA * tempRC * m_cos13) - Mathf.Pow(m_distAC, 2));

        print(lineNumber + " RA got this close to zero: " + gRA);
        lineNumber++;
        print(lineNumber + " RB got this close to zero: " + gRB);
        lineNumber++;
        print(lineNumber + " RC got this close to zero: " + gRC);
        lineNumber++;

        m_RA = tempRA; m_RB = tempRB; m_RC = tempRC;
        print(lineNumber + " RA: " + m_RA + " At " + iterations);
        lineNumber++;
        anglesReady = false;
    }

    private void updatePosition()
    {
        float x = Mathf.Abs(m_RA) * Mathf.Sin(m_B1) * Mathf.Cos(m_theta1);
        float y = Mathf.Abs(m_RA) * Mathf.Sin(m_B1) * Mathf.Sin(m_theta1);
        float z = Mathf.Abs(m_RA) * Mathf.Cos(m_B1);
        newPos = new Vector3(-(x - A_offsetX), -(z - A_offsetZ), -(y - A_offsetY));
    }

    // Update is called once per frame
    void Update()
    {

    }

     void test()
    {
        try
        {
            byte[] data = client.Receive(ref anyIP); string text = "";
            text = Encoding.ASCII.GetString(data); string[] splitArray = parseData(text);
            if (splitArray == null)
                return;
            print("UNVALIDATED: " + splitArray[0] + splitArray[1] + "" + splitArray[2] + splitArray[3]);
            lastReceivedUDPPacket = text; inputValidation(splitArray);
        }
        catch (Exception err)
        {
            print(err.ToString());
        } try
        {
            byte[] data = client.Receive(ref anyIP);
            string text = "";
            text = Encoding.ASCII.GetString(data); string[] splitArray = parseData(text);
            if (splitArray == null) return;
            print("UNVALIDATED: " + splitArray[0] + splitArray[1] + "" + splitArray[2] + splitArray[3]);
            lastReceivedUDPPacket = text;
            inputValidation(splitArray);
            print("b1 = " + m_B1 + " b2 = " + m_B2 + " b3 = " + m_B3);
            print("theta1 = " + m_theta1 + " theta2 = " + m_theta2 + " theta3 = " + m_theta3);
            extractAngles();
            NewtonsTestMethod();
            //updatePosition(); 
        }
        catch (Exception err)
        {
            print(err.ToString());
        }
    }
};

int _tmain(int argc, _TCHAR* argv[])
{
	return 0;
}

