#include "ros/ros.h"
#include <chrono>
#include <thread>
#include "geometry_msgs/Point.h"
#include "dynamixel_sdk/dynamixel_sdk.h"             // Uses Dynamixel SDK library

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_P_GAIN       			26
#define ADDR_MX_I_GAIN		          27
#define ADDR_MX_D_GAIN			     28
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting

#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define POS_MIN_1      				1448                // Dynamixel will rotate between this value
#define POS_MAX_1      				2648                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     5                  // Dynamixel moving status threshold

#define POS_MIN_2      				1050         
#define POS_MAX_2      				2050             
#define DXL_MOVING_STATUS_THRESHOLD     5  

#define ESC_ASCII_VALUE                 0x1b  
/*#define P_GAIN				          2 
#define I_GAIN			               1 
#define D_GAIN			               4*/
#define P_GAIN			               2
#define I_GAIN			               1 
#define D_GAIN			               1
using namespace std::this_thread;
using namespace std::chrono;

int DXL_ID[2]={1,2};
geometry_msgs::Point ball_position;

void tracker_callback(const geometry_msgs::Point msg){		
	ball_position=msg;
}
void buscarpelota();
void interpolar(int x, int y);

dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

int dxl_comm_result[2] = {COMM_TX_FAIL,COMM_TX_FAIL};             // Communication result
int dxl_goal_position[2] =	{2048,1550};         			// Goal position
int dxl_mid_position[2] =	{2048,1550};
uint8_t dxl_error[2] = {0,0};                          // Dynamixel error
uint16_t dxl_present_position[2] = {0,0};              // Present position

int main(int argc, char **argv){

	ros::init(argc, argv, "track");
	ros::NodeHandle n;
	
	ros::Subscriber tracker_sub = n.subscribe("position",10,tracker_callback);

	/************************************ INICIALIZAR MOTORES *********************************************/

	ball_position.x=320;
	ball_position.y=240;

	double dh, dv;

	// Open port
	if (portHandler->openPort()) {
		printf("Succeeded to open the port!\n");
	}
	else {
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		return 0;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE)) {
		printf("Succeeded to change the baudrate!\n");
	}
	else {
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		return 0;
	}

	// Enable Dynamixel Torque
	for(int i=0;i<2;i++){
		packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_P_GAIN, P_GAIN, &dxl_error[i]);
		packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_I_GAIN, I_GAIN, &dxl_error[i]);
		packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_D_GAIN, D_GAIN, &dxl_error[i]);
	}
		
	int comm_error=0;
	
	for(int i=0;i<2;i++){
		dxl_comm_result[i] = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error[i]);
		if (dxl_comm_result[i] != COMM_SUCCESS){
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result[i]));
			comm_error=1;
		}
		else if (dxl_error[i] != 0)	printf("%s\n", packetHandler->getRxPacketError(dxl_error[i]));
	}
	
	if (comm_error==0)			printf("Dynamixel has been successfully connected \n");

	
	while(ros::ok()){
		
		ros::spinOnce();
		
		//Si la pelota se pierde, voltear para todos lados
		if(ball_position.z==1)	buscarpelota();
		
		//Read present position
		for(int i=0; i<2; i++){
			dxl_comm_result[i] = packetHandler->read2ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_PRESENT_POSITION, &dxl_present_position[i], &dxl_error[i]);
		}	
		
		//Desplazamiento en pixeles
		dh = int(320 - ball_position.x);
		dv = int(240 - ball_position.y);
		
		//Realizando aproximaciones de conversión de pixeles a ángulo y de pixel a valor actuador, avanzar aproximadamente a 0.1 veces la posición necesaria
		//(para un movimiento suave)
		dxl_goal_position[0] = dxl_present_position[0] + dh;
		dxl_goal_position[1] = dxl_present_position[1] + dv;
		
		//Establecer límites de movimiento
		if(dxl_goal_position[0]<1024)	dxl_goal_position[0]=1024;
		if(dxl_goal_position[0]>3072)	dxl_goal_position[0]=3072;
		if(dxl_goal_position[1]<1060)	dxl_goal_position[1]=1060;
		if(dxl_goal_position[1]>2030)	dxl_goal_position[1]=2030;
		
		//Mandar posición al motor
		dxl_comm_result[0] = packetHandler->write2ByteTxRx(portHandler, DXL_ID[0], ADDR_MX_GOAL_POSITION, dxl_goal_position[0], &dxl_error[0]);
		dxl_comm_result[1] = packetHandler->write2ByteTxRx(portHandler, DXL_ID[1], ADDR_MX_GOAL_POSITION, dxl_goal_position[1], &dxl_error[1]);
	
	}

	// Disable Dynamixel Torque
	for(int i=0; i<2; i++){
			dxl_comm_result[i] = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error[i]);
		}

	// Close port
	portHandler->closePort();

	return 0;
}


void buscarpelota(){
	sleep_for(milliseconds(40));
	//Girar para todos lados
	while(ball_position.z!=0||!ros::ok()){
		ros::spinOnce();
		interpolar(2048,1800);
		interpolar(3072,1800);
		interpolar(2048,1800);
		interpolar(1024,1800);
	}
	sleep_for(milliseconds(50));
}

void interpolar(int x, int y){
	
	//Código para interpolación de quinto orden
	
	if(ball_position.z==0||!ros::ok()) return;
	double tf = 200;
	double a5 = 6/pow(tf,5);
	double a4 = -2.5*a5*tf;
	double a3 = 1.666667*a5*pow(tf,2);
	double w = 0;
	
	dxl_goal_position[0]=x;
	dxl_goal_position[1]=y;
	
	// Read current position
	for(int i=0; i<2; i++){
		packetHandler->read2ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_PRESENT_POSITION, &dxl_present_position[i], &dxl_error[i]);
	}
	
	// Write goal position using quintic interpolation
	for(int t=0;t<=tf;t++){
		w = a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5);
		ros::spinOnce();
		if(ball_position.z==0||!ros::ok()) break;
		for(int i=0;i<2;i++){
			dxl_mid_position[i] = dxl_present_position[i] + w * (dxl_goal_position[i] - dxl_present_position[i]);
			packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_GOAL_POSITION, dxl_mid_position[i], &dxl_error[i]);
		}
		sleep_for(milliseconds(2));
	}
}
