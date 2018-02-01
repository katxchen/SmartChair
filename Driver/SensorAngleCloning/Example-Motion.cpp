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
#include "stdlib.h"

#include <ctime>
#include <time.h>


using namespace sFnd;
using std::cout;
using std::endl;

/*Portname must contain these backslashes, and remember to
replace the following com port*/
char *port_name = "\\\\.\\COM7";

//String for incoming data
char incomingData[MAX_DATA_LENGTH];
int incomingFigure;


//*********************************************************************************
//This program will load configuration files onto each node connected to the port, then executes
//sequential repeated moves on each axis.
//*********************************************************************************

#define PORT_NUM			4	//The port's COM number (as seen in device manager)
#define ACC_LIM_RPM_PER_SEC	100000
#define VEL_LIM_RPM			700
//#define MOVE_DISTANCE_CNTS	40	
#define NUM_MOVES			10000

int main(int argc, char* argv[])
{
	SerialPort arduino(port_name);
	if (arduino.isConnected()) cout << "Arduino Connection Established" << endl;
	else cout << "ERROR, check Arduino port name";

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
	try {
		for (size_t i = 0; i < NUM_MOVES; i++)
		{
			for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
				// Create a shortcut reference for a node
				INode &theNode = myPort.Nodes(iNode);

				//int current_angle = theNode.Motion.PosnMeasured;
				//theNode.Motion.PosnMeasured.Refresh();		//Refresh our current measured position


				//Check if data has been read or not
				int read_result = arduino.readSerialPort(incomingData, MAX_DATA_LENGTH);
				std::string res(incomingData);
				int begin = res.find_first_of(".");
				int end = res.find_first_of(",", begin + 1);
				if (begin != -1 && end != -1) {
					res = res.substr(begin + 1, end - 1);
					//std::cout << "Res: " << res << std::endl;
					long command = atoi(res.c_str())/500;
					double com = command / 10000.0;
					double time_diff = (double)(clock() - start_time) / CLOCKS_PER_SEC;
					start_time = clock();
					printf("command: %f\t\t res: %s\t\t Read: %s\t\t time: %f\n", com, res.c_str(), incomingData, time_diff);
					//comm.push_back(com);
					//duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
					//time.push_back(duration);
					//pos++;
					/*if (pos > MAX_PLOT_ENTRIES) {
						//plt::clf();
						//cout << "deleting" << endl;
						comm.erase(comm.begin());
						time.erase(time.begin());
					}*/
					//plt::plot(time, comm, "r");
					//plt::show();
					//plt::pause(0.000001);
					//int current_angle2 = theNode.Motion.PosnMeasured;
					//prints out data
					//puts(incomingData);
					//std::string test(incomingData);
					//printf("Read Result \t%i \n", incomingData);
					//Sleep(10);

					//print
					//printf("Read Result \t%i %d\n", read_result, );
					//std::cout << "current angle " << current_angle << std::endl;
					//std::cout << "current angle2 " << current_angle2 << std::endl;
					//std::cout << "Return code: " << read_result << std::endl;
					//std::cout << "Read Result " << incomingData << std::endl;
					//int command = atoi(incomingData) * 50;
					int safetyValue = 1000;

					if (command > safetyValue)
					{
						std::cout << "Command " << command << " is too big" << std::endl;
						command = safetyValue;
					}
					else if (command < -safetyValue)
					{
						std::cout << "Command " << command << " is too small" << std::endl;
						command = -safetyValue;
					}
					std::cout << "Casted command value: " << command << std::endl;


					theNode.Motion.MoveWentDone();						//Clear the rising edge Move done register

					theNode.AccUnit(INode::RPM_PER_SEC);				//Set the units for Acceleration to RPM/SEC
					theNode.VelUnit(INode::RPM);						//Set the units for Velocity to RPM
					theNode.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;		//Set Acceleration Limit (RPM/Sec)
					theNode.Motion.VelLimit = VEL_LIM_RPM;				//Set Velocity Limit (RPM)

					printf("Moving Node \t%i \n", iNode);

					theNode.Motion.MovePosnStart(command, true);			//Execute angle encoder count move 

					double timeout = myMgr.TimeStampMsec() + theNode.Motion.MovePosnDurationMsec(command) + 50;			//define a timeout in case the node is unable to enable


					while (!theNode.Motion.MoveWentDone()) {
						if (myMgr.TimeStampMsec() > timeout) {
							printf("Error: Timed out waiting for move to complete\n");
							return -2;
						}
					}
					printf("Node \t%i Move Done\n", iNode);
				}
			}
		//myMgr.Delay(5);
		}
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