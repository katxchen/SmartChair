//*****************************************************************************
// $Archive: /ClearPath SC/Installer/Program Files/Teknic/ClearView/sdk/examples/4-Example-Motion/Example-Motion.cpp $
// $Revision: 3 $ $Date: 11/22/16 3:49p $
// $Workfile: Example-Motion.cpp $
//*****************************************************************************

//Required include files
#include "stdio.h"
#include <string>
#include "string.h"
#include "pubSysCls.h"	

//----------------------------------------------
#include "SerialPort.h"
#include <iostream>
#include <fstream>
#include <iomanip>

#include "stdlib.h"

#include <ctime>
#include <time.h>

#include <numeric>
#include <vector>

using namespace std;
using namespace sFnd;
using std::cout;
using std::endl;

/*Portname must contain these backslashes, and remember to
replace the following com port*/
char *port_name = "\\\\.\\COM7";

//String for incoming data
char incomingData[MAX_DATA_LENGTH];
int incomingFigure;

char* filename1 = "D:\\test1.csv";

double intercept = 0.0;
double slope = 0.0;


//*********************************************************************************
//This program will load configuration files onto each node connected to the port, then executes
//sequential repeated moves on each axis.
//*********************************************************************************

#define PORT_NUM			4	//The port's COM number (as seen in device manager)
#define ACC_LIM_RPM_PER_SEC	10000
#define VEL_LIM_RPM			500
//#define MOVE_DISTANCE_CNTS	40	
#define NUM_MOVES			10000

#define SAFETY_VALUE 2200

#define NUM_CAL_ITER 3
#define CAL_VEL 150.0

#define CENTER_GAIN 100.0

#define ANGLE_DIVISOR 10000.0

#define ACC_ANG_MODE '0'
#define GRAV_ANG_MODE '1'
#define ORIENTATION_MODE '2'

void clearInc() {
	for (int i = 0; i < MAX_DATA_LENGTH; i++) {
		incomingData[i] = 0;
	}
}

int zeroPosition(INode& theNode, SysManager myMgr) {
	theNode.Motion.MoveWentDone();						//Clear the rising edge Move done register		
	theNode.Motion.MovePosnStart(0, true);
	double timeout = myMgr.TimeStampMsec() + theNode.Motion.MovePosnDurationMsec(0) + 120;			//define a timeout in case the node is unable to enable

	while (!theNode.Motion.MoveWentDone()) {
		if (myMgr.TimeStampMsec() > timeout) {
			printf("Error: Timed out waiting for move to complete\n");
			return -2;
		}
	}
	printf("Moved to 0 Position\n");
	return 0;
}

long getArduinoData(SerialPort arduino) {
	int read_result = arduino.readSerialPort(incomingData, MAX_DATA_LENGTH);
	std::string res(incomingData);
	//cout << "res: " << res << endl;
	int begin = res.find_last_of(".");
	int end = res.find_first_of(",", begin + 1);
	if (end == -1) {
		res = res.substr(0, begin);
	}
	begin = res.find_last_of(".");
	end = res.find_first_of(",", begin + 1);
	if (begin != -1 && end != -1) {
		res = res.substr(begin + 1, end - 1);
		std::cout << "Res: " << res << std::endl;
		return atoi(res.c_str());
	}
	return LONG_MIN;
}

double getRealAng(long angle) {
	return ((double)angle / ANGLE_DIVISOR);
}

double linReg(vector<double>& x,vector<double>& y) {
	if (x.size() != y.size()) {
		throw exception("...");
	}
	double n = x.size();

	double avgX = std::accumulate(x.begin(), x.end(), 0.0) / n;
	double avgY = std::accumulate(y.begin(), y.end(), 0.0) / n;

	double numerator = 0.0;
	double denominator = 0.0;

	for (int i = 0; i<n; ++i) {
		numerator += (x[i] - avgX) * (y[i] - avgY);
		denominator += (x[i] - avgX) * (x[i] - avgX);
	}

	if (denominator == 0) {
		throw exception("...");
	}

	slope = numerator / denominator;
	intercept = avgY - (slope*avgX);
}

int main(int argc, char* argv[])
{
	SerialPort arduino(port_name);
	if (arduino.isConnected()) cout << "Arduino Connection Established" << endl;
	else cout << "ERROR, check Arduino port name";

	clearInc();
	ofstream fp;
	fp.open(filename1);

	//These represent to locations of any and all config files to be loaded into the motors.  Uncomment line 96 to load configs.
	std::string CONFIG_FILES[] = { "C:\\filepath\\Axis0.mtr",
								   "C:\\filepath\\Axis1.mtr",
								   "C:\\filepath\\Axis2.mtr",
								   "C:\\filepath\\Axis3.mtr" };


												//Create the SysManager object.  This will object will coordinate actions among various ports,
												// and within nodes.  In this example we use this object to setup and open our port.
	SysManager myMgr;							//Create System Manager myMgr

	printf("Hello World, I am SysManager\n");	//Print to console

	printf("\n I will now open port \t%i \n \n", PORT_NUM);

	//This will try to open the port.  If there is an error/exception during the port opening
	//the code will jump to the catch loop where detailed information regarding the error will be displayed,
	//otherwise the catch loop is skipped over
	try
	{
		myMgr.ComHubPort(0, PORT_NUM); 	//define the first SC Hub port (port 0) to be associated with COM 
										//PORT_NUM  (as seen in device manager)

		myMgr.PortsOpen(1);				//Open the first port.
	}
	catch (mnErr theErr)	//This catch statement will intercept any error from the Class library
	{
		printf("Port Failed to open, shutting down\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		::system("pause"); //pause so the user can see the error message; waits for user to press a key

		return 0;  //This terminates the main program
	}

	//Once the code gets past this point, it can be assumed that the Port has been opened without issue
	//Now we can get a reference to our port object which we will use to access the node objects

	IPort &myPort = myMgr.Ports(0);		//set the reference myPort to refer to port 0 

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Here we ensure that the port has the proper number of nodes and loads the stored config files onto each node
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	if(sizeof(CONFIG_FILES)/sizeof(CONFIG_FILES[0]) < myPort.NodeCount()) {

		printf("Wrong number of nodes detected\n");
		printf("I have  \t%i  Nodes \n \n", myPort.NodeCount());  //Print how many nodes the port has

		::system("pause"); //pause so the user can see the error message; waits for user to press a key

		return -1;  //This terminates the main program
}
	
	try {
		for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
			// Create a shortcut reference for a node
			INode &theNode = myPort.Nodes(iNode);

			theNode.EnableReq(false);				//Ensure Node is disabled before loading config file

			myMgr.Delay(200);


			//Uncomment To load saved config files before motion
			//theNode.Setup.ConfigLoad(CONFIG_FILES[iNode].c_str());
			
			printf("   Node[%d]: type=%d\n", int(iNode), theNode.Info.NodeType());
			printf("            userID: %s\n", theNode.Info.UserID.Value());
			printf("        FW version: %s\n", theNode.Info.FirmwareVersion.Value());
			printf("          Serial #: %d\n", theNode.Info.SerialNumber.Value());
			printf("             Model: %s\n", theNode.Info.Model.Value());

			//The following statements will attempt to enable the node.  First,
			// any shutdowns or NodeStops are cleared, finally the node in enabled
			theNode.Status.AlertsClear();					//Clear Alerts on node 
			theNode.Motion.NodeStopClear();	//Clear Nodestops on Node  				
			theNode.EnableReq(true);					//Enable node 

			//theNode.Motion.PosnMeasured.Refresh();		//Refresh our current measured position


			double timeout = myMgr.TimeStampMsec() + 3000;			//define a timeout in case the node is unable to enable
															//This will loop checking on the Real time values of the node's Ready status
			while (!theNode.Motion.IsReady()) {
				if (myMgr.TimeStampMsec() > timeout) {
					printf("Error: Timed out waiting for Node \t%i to enable\n", iNode);
					return -2;
				}
			}
			//At this point the node is enabled
			printf("Node \t%i enabled\n", iNode);
		}
	}
	catch (mnErr theErr)
	{
		printf("Failed to load config files n\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);


		return 0;  //This terminates the main program
	}


	///////////////////////////////////////////////////////////////////////////////////////
	//At this point we will execute 10 rev moves sequentially on each axis
	//////////////////////////////////////////////////////////////////////////////////////

	double duration;


	//int n = 500;
	std::vector<double> comm, time;
	int pos = 0;
	//plt::ion();
	clock_t start_time = clock();
	clock_t overall_start_time = clock();
	myPort.Nodes(0).AccUnit(INode::RPM_PER_SEC);				//Set the units for Acceleration to RPM/SEC
	myPort.Nodes(0).VelUnit(INode::RPM);						//Set the units for Velocity to RPM
	myPort.Nodes(0).Motion.AccLimit = ACC_LIM_RPM_PER_SEC;		//Set Acceleration Limit (RPM/Sec)
	myPort.Nodes(0).Motion.VelLimit = VEL_LIM_RPM;				//Set Velocity Limit (RPM)
	long currPos = 0.0;
	int dir = 1;
	bool centered = false;
	int read_result = arduino.readSerialPort(incomingData, MAX_DATA_LENGTH);
	myMgr.Delay(500);
	read_result = arduino.readSerialPort(incomingData, MAX_DATA_LENGTH);
	myMgr.Delay(500);
	//getArduinoData(arduino);
	try {
		INode &theNode = myPort.Nodes(0);
		printf("CALIBRATING...\n");
		printf("Finding 0 position...\n");
		while (!centered) {
			long angle;
			clearInc();
			int read_result = arduino.readSerialPort(incomingData, MAX_DATA_LENGTH);
			std::string res(incomingData);
			//cout << "res: " << res << endl;
			int begin = res.find_last_of(".");
			int end = res.find_first_of(",", begin + 1);
			if (end == -1) {
				res = res.substr(0, begin);
			}
			begin = res.find_last_of(".");
			end = res.find_first_of(",", begin + 1);
			if (begin != -1 && end != -1) {
				res = res.substr(begin + 1, end - 1);
				//std::cout << "Res: " << res << std::endl;
				angle = atoi(res.c_str());
			}
			else {
				angle = LONG_MIN;
			}
			double realAng = getRealAng(angle);
			cout << "angle: " << realAng << endl;
			theNode.Motion.MoveWentDone();
			theNode.Motion.PosnMeasured.Refresh();
			currPos = theNode.Motion.PosnMeasured.Value();
			if (currPos > SAFETY_VALUE) {
				theNode.Motion.MoveVelStart(-CENTER_GAIN);
			}
			else if (currPos < -SAFETY_VALUE) {
				theNode.Motion.MoveVelStart(CENTER_GAIN);
			}else if (angle != LONG_MIN) {
				theNode.Motion.MoveVelStart(-CENTER_GAIN * realAng);
			}
			if (abs(realAng) < 0.1) {
				theNode.Motion.NodeStop(STOP_TYPE_ABRUPT);
				//theNode.Motion.MoveVelStart(0.0);
				theNode.Motion.AddToPosition(-(double)theNode.Motion.PosnMeasured);
				centered = true;
				char *c_string = new char[1];
				c_string[0] = ORIENTATION_MODE;
				// Tell the arduino to reset and change to absolute orientation mode
				arduino.writeSerialPort(c_string, 1);
				myMgr.Delay(1000);
				int read_result = arduino.readSerialPort(incomingData, MAX_DATA_LENGTH);
				std::string res(incomingData);
				cout << "res: " << res << endl;
				printf("found 0 position!\n");
				myMgr.Delay(1000);

			}
			myMgr.Delay(10);
		}
		myMgr.Delay(1000);

		vector<double> pos;
		vector<double> val;
		printf("Calibrating motion...\n");
		for (int i = 0; i < NUM_CAL_ITER; i++) {
			bool goingToCenter = false;
			bool calibrated = false;
			bool started = false;
			while (!calibrated)
			{
				if (!started) {
					printf("Zeroing position...\n");
					theNode.Motion.NodeStopClear();
					theNode.Motion.MoveWentDone();						//Clear the rising edge Move done register		
					theNode.Motion.MovePosnStart(0, true);
					double timeout = myMgr.TimeStampMsec() + theNode.Motion.MovePosnDurationMsec(0) + 120;			//define a timeout in case the node is unable to enable

					while (!theNode.Motion.MoveWentDone()) {
						if (myMgr.TimeStampMsec() > timeout) {
							printf("Error: Timed out waiting for move to complete\n");
							return -2;
						}
					}
					theNode.Motion.MoveWentDone();
					theNode.Motion.MoveVelStart(CAL_VEL);
					started = true;
				}

				theNode.Motion.PosnMeasured.Refresh();
				currPos = theNode.Motion.PosnMeasured.Value();

				if (currPos > SAFETY_VALUE)
				{
					theNode.Motion.MoveWentDone();
					std::cout << "Reached Max Value, moving other direction" << std::endl;
					theNode.Motion.MoveVelStart(-CAL_VEL);
				}
				else if (currPos < -SAFETY_VALUE)
				{
					theNode.Motion.MoveWentDone();
					std::cout << "Reached Min Value, Returning to Center" << std::endl;
					theNode.Motion.MoveWentDone();						//Clear the rising edge Move done register		
					theNode.Motion.MoveVelStart(CAL_VEL);
					goingToCenter = true;
				}
				if (goingToCenter) {
					theNode.Motion.PosnMeasured.Refresh();
					if (theNode.Motion.PosnMeasured.Value() > 0.0) {
						theNode.Motion.MoveWentDone();
						std::cout << "Reached Center" << std::endl;
						theNode.Motion.MovePosnStart(CAL_VEL);
						double timeout = myMgr.TimeStampMsec() + theNode.Motion.MovePosnDurationMsec(0) + 200;			//define a timeout in case the node is unable to enable

						while (!theNode.Motion.MoveWentDone()) {
							if (myMgr.TimeStampMsec() > timeout) {
								printf("Error: Timed out waiting for move to complete\n");
								return -2;
							}
						}
						calibrated = true;
						cout << "Calibrated" << endl;
					}
				}

				//Check if data has been read or not
				long angle;
				clearInc();
				int read_result = arduino.readSerialPort(incomingData, MAX_DATA_LENGTH);
				std::string res(incomingData);
				//cout << "res: " << res << endl;
				int begin = res.find_last_of(".");
				int end = res.find_first_of(",", begin + 1);
				if (end == -1) {
					res = res.substr(0, begin);
				}
				begin = res.find_last_of(".");
				end = res.find_first_of(" ", begin + 1);
				if (begin != -1 && end != -1) {
					res = res.substr(begin + 1, end - 1);
					//std::cout << "Res: " << res << std::endl;
					angle = atoi(res.c_str());
				}
				else {
					angle = LONG_MIN;
				}
				if (angle != LONG_MIN) {
					double realAng = getRealAng(angle);
					theNode.Motion.PosnMeasured.Refresh();
					fp << theNode.Motion.PosnMeasured.Value() << "," << realAng << endl;
					cout << theNode.Motion.PosnMeasured.Value() << "," << realAng << endl;
					pos.push_back(theNode.Motion.PosnMeasured.Value());
					val.push_back(realAng);
				}
				myMgr.Delay(8);
			}
		}
		printf("Calibration Finished!\nCalculating Linear Regression...\n");
		linReg(pos, val);
		printf("The formula for the regression line is:\nAngle = %f * position + %f\n", slope, intercept);
		fp.close();
	}
		catch (mnErr theErr)
		{
			printf("Error during moves n\n");
			//This statement will print the address of the error, the error code (defined by the mnErr class), 
			//as well as the corresponding error message.
			printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

			::system("pause"); //pause so the user can see the error message; waits for user to press a key

			return 0;  //This terminates the main program
		}
	
		//////////////////////////////////////////////////////////////////////////////////////////////
		//After moves have completed Disable node, and close ports
		//////////////////////////////////////////////////////////////////////////////////////////////
		printf("Disabling nodes, and closing port\n");
		//Disable Nodes
		try {
			for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
				// Create a shortcut reference for a node
				myPort.Nodes(iNode).Outs.EnableReq(false);
			}
		}
		catch (mnErr theErr)
		{
			printf("Failed to disable Nodes n\n");
			//This statement will print the address of the error, the error code (defined by the mnErr class), 
			//as well as the corresponding error message.
			printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

			::system("pause"); //pause so the user can see the error message; waits for user to press a key

			return 0;  //This terminates the main program
		}

	// Close down the ports
	myMgr.PortsClose();

	::system("pause"); //pause so the user can see the program output; waits for user to press a key
	return 0;			//End program
}